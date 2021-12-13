/*
 * Copyright (c) 2018, Bertold Van den Bergh (vandenbergh@bertold.org)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the author nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR DISTRIBUTOR BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "ESPZoneDetect.hpp"
#include "tzData.hpp"

#include <algorithm>
#include <array>
#include <cstring>
#include <iterator>

// --- public ---

ESPZoneDetect::ESPZoneDetect() {
}

ESPZoneDetect::~ESPZoneDetect() {
  m_cleanUp();
}

bool ESPZoneDetect::OpenDatabaseFromMemory(void* buffer, const size_t length) {
  if (m_length = (uint32_t)length; m_length == 0) {
    m_zdErrorHandler(ZD_E_DB_SEEK, 0);
    return false;
  }

  m_mapping->setMemory((uint8_t*)buffer);

  /* Parse the header */
  if (ParseHeader() != ZD_LOOKUP_PARSE_OK) {
    m_zdErrorHandler(ZD_E_PARSE_HEADER, 0);
    return false;
  }

  return true;
}

bool ESPZoneDetect::OpenDatabase(fs::File* fd) {
  if (m_length = fd->size(); m_length == 0) {
    m_zdErrorHandler(ZD_E_DB_SIZE, errno);
    return false;
  }

  if (m_length > 50331648) {
    m_zdErrorHandler(ZD_E_DB_HUGE, errno);
    return false;
  }

  if (!fd->seek(0, SeekSet)) {
    m_zdErrorHandler(ZD_E_DB_SEEK, errno);
    return false;
  }

  m_mapping->setFile(fd);

  /* Parse the header */
  if (ParseHeader() != ZD_LOOKUP_PARSE_OK) {
    m_zdErrorHandler(ZD_E_PARSE_HEADER, 0);
    return false;
  }

  return true;
}

std::pair<std::vector<ESPZoneDetect::ZoneDetectResult>, double>
ESPZoneDetect::Lookup(const double lat, const double lon) const {
  const auto latFixedPoint{DoubleToFixedPoint(lat, 90)};
  const auto lonFixedPoint{DoubleToFixedPoint(lon, 180)};
  uint64_t distanceSqrMin{(uint64_t)-1};

  /* Iterate over all polygons */
  uint32_t bboxIndex{m_bboxOffset}, metadataIndex{0},
           polygonIndex{0}, polygonId{0};

  std::vector<ZoneDetectResult> results;

  while (bboxIndex < m_metadataOffset) {
    int32_t minLat, minLon, maxLat, maxLon, metadataIndexDelta;
    uint64_t polygonIndexDelta;
    if (!DecodeVariableLengthSigned(bboxIndex, minLat)) break;
    if (!DecodeVariableLengthSigned(bboxIndex, minLon)) break;
    if (!DecodeVariableLengthSigned(bboxIndex, maxLat)) break;
    if (!DecodeVariableLengthSigned(bboxIndex, maxLon)) break;
    if (!DecodeVariableLengthSigned(bboxIndex, metadataIndexDelta)) break;
    if (!DecodeVariableLengthUnsigned(bboxIndex, polygonIndexDelta)) break;

    metadataIndex += metadataIndexDelta;
    polygonIndex += (uint32_t)polygonIndexDelta;

    if (latFixedPoint >= minLat) {
      if (latFixedPoint <= maxLat && lonFixedPoint >= minLon &&
          lonFixedPoint <= maxLon) {
        const auto [lookupResult, distanceSqrMin]{PointInPolygon(
            m_dataOffset + polygonIndex, latFixedPoint, lonFixedPoint)};
        if (lookupResult == ZD_LOOKUP_PARSE_ERROR) { break; }
        if (lookupResult != ZD_LOOKUP_NOT_IN_ZONE) {
          results.emplace_back(ZoneDetectResult{
              .lookupResult = lookupResult,
              .polygonId = polygonId,
              .metaId = metadataIndex,
          });
        }
      }
    } else {
      /* The data is sorted along minLat */
      break;
    }

    polygonId++;
  }

  /* Clean up results */
  for (auto& result0 : results) {
    uint32_t insideSum{0};
    auto overrideResult { ZD_LOOKUP_IGNORE };
    for (auto& result1 : results) {
      if (result0.metaId == result1.metaId) {
        /* This is the same result. Is it an exclusion zone? */
        switch (result1.lookupResult) {
          case ZD_LOOKUP_IN_ZONE:
            insideSum++;
            break;
          case ZD_LOOKUP_IN_EXCLUDED_ZONE:
            insideSum--;
            break;
          default:
            overrideResult = result1.lookupResult;
            break;
        }
        result1.lookupResult = ZD_LOOKUP_IGNORE;
      }
    }

    if (overrideResult != ZD_LOOKUP_IGNORE) {
      result0.lookupResult = overrideResult;
      continue;
    }
    if (insideSum) {
        result0.lookupResult = ZD_LOOKUP_IN_ZONE;
    }
  }

  /* Remove zones to ignore */
  results.erase(std::remove_if(results.begin(), results.end(),
                               [](ZoneDetectResult const& r) {
                                 return !(r.lookupResult != ZD_LOOKUP_IGNORE);
                               }),
                results.end());

  /* Lookup metadata */
  for (auto& result : results) {
    uint32_t tmpIndex{m_metadataOffset + result.metaId};
    for (const auto& name : m_fieldNames) {
      if (auto data = ParseString(tmpIndex); !data.empty()) {
        result.fields[name] = data;
        continue;
      }
      return {std::vector<ZoneDetectResult>{}, 0};
    }
  }

  /* Write end marker */
  results.emplace_back(ZoneDetectResult{
      .lookupResult = ZD_LOOKUP_END,
  });

  return {results, sqrt((double)distanceSqrMin) * 90.0 /
                   (double)(1 << (m_precision - 1))};
}

std::string ESPZoneDetect::LookupName(
    const double lat, const double lon) const {
  auto [result, safezone]{Lookup(lat, lon)};
  if (result.empty() || result[0].lookupResult == ZD_LOOKUP_END) {
    return "";
  }

  std::string resultString{""};
  const auto& fields { result[0].fields };
  switch (GetTableType()) {
    case 'T':
      if (auto found{fields.find("TimezoneIdPrefix")}; found != fields.end()) {
        resultString += found->second;
      }
      if (auto found{fields.find("TimezoneId")}; found != fields.end()) {
        resultString += found->second;
      }
      break;
    case 'C':
      if (auto found{fields.find("Name")}; found != fields.end()) {
        resultString = found->second;
      }
      break;
    default:
      break;
  }

  return resultString;
}

std::string ESPZoneDetect::LookupPosix(const double lat, const double lon) const {
  auto tzName { LookupName(lat, lon) };
  return getPosix(tzName);
}

std::array<std::string, 2> ESPZoneDetect::LookupBoth(const double lat, const double lon) const {
  auto tzName { LookupName(lat, lon) };
  return { tzName, getPosix(tzName) };
}

std::string ESPZoneDetect::getPosix(const std::string& tzName) const {
  if (!tzName.empty()) {
    char tzStr[128];
    for (auto const& tz : tzData) {
      strcpy_P(tzStr, tz);
      if (auto colon = strchr(tzStr, ':'); colon) {
        *colon++ = '\0';
        if (tzName == tzStr) return colon;
      }
    }
  }
  return "" ;
}

const char* ESPZoneDetect::LookupResultToString(ZDLookupResult result) const {
  static std::unordered_map<ZDLookupResult, const char*> resultMap {
    { ZD_LOOKUP_IGNORE,             "Ignore" },
    { ZD_LOOKUP_END,                "End" },
    { ZD_LOOKUP_PARSE_ERROR,        "Parsing error" },
    { ZD_LOOKUP_NOT_IN_ZONE,        "Not in zone" },
    { ZD_LOOKUP_IN_ZONE,            "In zone" },
    { ZD_LOOKUP_IN_EXCLUDED_ZONE,   "In excluded zone" },
    { ZD_LOOKUP_ON_BORDER_VERTEX,   "Target point is border vertex" },
    { ZD_LOOKUP_ON_BORDER_SEGMENT,  "Target point is on border" },
    { ZD_LOOKUP_PARSE_OK,           "Parsing Successful" },
  };

  auto found{resultMap.find(result)};
  return (found != resultMap.end()) ? found->second : "Unknown";
}

#define ZD_E_COULD_NOT(msg) "could not " msg
const char* ESPZoneDetect::GetErrorString(ZDInternalError errZD) const {
  static std::unordered_map<ZDInternalError, const char*> errorMap{
      {ZD_E_DB_OPEN,      ZD_E_COULD_NOT("open database file")},
      {ZD_E_DB_SIZE,      ZD_E_COULD_NOT("retrieve database file size")},
      {ZD_E_DB_HUGE,      "database file is too large"},
      {ZD_E_DB_SEEK,      ZD_E_COULD_NOT("seek database file")},
      {ZD_E_DB_MMAP,      ZD_E_COULD_NOT("map database file to system memory")},
      {ZD_E_DB_MUNMAP,    ZD_E_COULD_NOT("unmap database")},
      {ZD_E_DB_CLOSE,     ZD_E_COULD_NOT("close database file")},
      {ZD_E_PARSE_HEADER, ZD_E_COULD_NOT("parse database header")},
  };

  auto found{errorMap.find(errZD)};
  return (found != errorMap.end()) ? found->second : "unknown error";
}
#undef ZD_E_COULD_NOT

void ESPZoneDetect::SetErrorHandler(zdErrorHandler_t handler) {
  m_zdErrorHandler = handler;
}

void ESPZoneDetect::SetCleanUp(zdCleanup_t cleanUp) {
  m_cleanUp = cleanUp;
}

uint8_t ESPZoneDetect::GetTableType() const { return m_tableType; }
const char* ESPZoneDetect::GetNotice() const { return m_notice.c_str(); }

// ---- private ----

int32_t ESPZoneDetect::DoubleToFixedPoint(const double input, const double scale) const {
  return (int32_t)((input / scale) * (double)(1 << (m_precision - 1)));
}

double ESPZoneDetect::FixedPointToDouble(const int32_t input, const double scale) const {
  return ((double)input / (double)(1 << (m_precision - 1))) * scale;
}

bool ESPZoneDetect::DecodeVariableLengthUnsigned(
    uint32_t& index, uint64_t& result) const {
  if (index >= m_length) {
    return false;
  }

  uint64_t value { 0 };
  uint32_t i { 0 }, shift { 0 };
  while (true) {
    value |= ((*m_mapping->atUint64(index + i) & UINT8_C(0x7F)) << shift);
    shift += (uint32_t)7;

    if (!(*m_mapping->at(index + i) & UINT8_C(0x80))) { break; }

    i++;
    if (index + i > m_length - 1) { return false; }
  }

  result = value;
  index += ++i;
  return true;
}

bool ESPZoneDetect::DecodeVariableLengthUnsignedReverse(
    uint32_t& index, uint64_t& result) const {
  if (index == 0 || index >= m_length || *m_mapping->at(index) & UINT8_C(0x80)) {
    return false;
  }

  uint32_t i{ index - 1 };
  for (; i != 0 && *m_mapping->at(i) & UINT8_C(0x80); i--);
  if (i == 0) { return false; }

  index = i;
  uint32_t i2 = ++i;
  return DecodeVariableLengthUnsigned(i2, result);
}

int64_t ESPZoneDetect::DecodeUnsignedToSigned(const uint64_t value) const {
  return (value & 1) ? -(int64_t)(value / 2) : (int64_t)(value / 2);
}

bool ESPZoneDetect::DecodeVariableLengthSigned(
    uint32_t& index, int32_t& result) const {
  uint64_t value;
  const auto retVal {
      DecodeVariableLengthUnsigned(index, value) };
  result = (int32_t)DecodeUnsignedToSigned(value);
  return retVal;
}

std::string ESPZoneDetect::ParseString(uint32_t& index) const {
  uint64_t strLength;
  if (!DecodeVariableLengthUnsigned(index, strLength)) {
    return "";
  }

  uint32_t strOffset { index };
  bool remoteStr { false };
  if (strLength >= 256) {
    strOffset = m_metadataOffset + (uint32_t)strLength - 256;
    remoteStr = true;

    if (!DecodeVariableLengthUnsigned(strOffset, strLength) ||
        strLength > 256) {
      return "";
    }
  }

  std::string str;
  str.reserve(strLength);
  if (str.capacity() < strLength) { return ""; }

  for (uint64_t i = 0; i < strLength; i++) {
    str.push_back((char)*m_mapping->at(strOffset + i) ^ UINT8_C(0x80));
  }

  if (!remoteStr) {
    index += (uint32_t)strLength;
  }

  return str;
}

ESPZoneDetect::ZDLookupResult ESPZoneDetect::ParseHeader() {
  if (m_length < 7 || memcmp(m_mapping->at(0), "PLB", 3)) {
    return ZD_LOOKUP_PARSE_ERROR;
  }

  uint32_t index{3};
  m_tableType = *m_mapping->at(index++);
  uint8_t version = *m_mapping->at(index++);
  m_precision = *m_mapping->at(index++);
  uint8_t numFields = *m_mapping->at(index++);

  // only support version 1
  m_fieldNames.reserve(numFields);
  if (version != 1 || m_fieldNames.capacity() < numFields) {
    return ZD_LOOKUP_PARSE_ERROR;
  }

  for (size_t i = 0; i < numFields; i++) {
    m_fieldNames.emplace_back(ParseString(index));
  }

  m_notice = ParseString(index);
  if (m_notice.empty()) {
    return ZD_LOOKUP_PARSE_ERROR;
  }

  uint64_t tmp;
  /* Read section sizes */
  /* By memset: bboxOffset = 0 */

  if (!DecodeVariableLengthUnsigned(index, tmp))
    return ZD_LOOKUP_PARSE_ERROR;
  m_metadataOffset = (uint32_t)tmp + m_bboxOffset;

  if (!DecodeVariableLengthUnsigned(index, tmp))
    return ZD_LOOKUP_PARSE_ERROR;
  m_dataOffset = (uint32_t)tmp + m_metadataOffset;

  if (!DecodeVariableLengthUnsigned(index, tmp))
    return ZD_LOOKUP_PARSE_ERROR;

  /* Add header size to everything */
  m_bboxOffset += index;
  m_metadataOffset += index;
  m_dataOffset += index;

  /* Verify file length */
  if (tmp + m_dataOffset != m_length) {
    return ZD_LOOKUP_END;
  }

  return ZD_LOOKUP_PARSE_OK;
}

bool ESPZoneDetect::PointInBox(const int32_t xl, const int32_t x, const int32_t xr,
                               const int32_t yl, const int32_t y, const int32_t yr) const {
  return ((xl <= x && x <= xr) || (xr <= x && x <= xl)) &&
         ((yl <= y && y <= yr) || (yr <= y && y <= yl));
}

uint32_t ESPZoneDetect::Unshuffle(uint64_t w) const {
  w &= 0x5555555555555555llu;
  w = (w | (w >> 1)) & 0x3333333333333333llu;
  w = (w | (w >> 2)) & 0x0F0F0F0F0F0F0F0Fllu;
  w = (w | (w >> 4)) & 0x00FF00FF00FF00FFllu;
  w = (w | (w >> 8)) & 0x0000FFFF0000FFFFllu;
  w = (w | (w >> 16)) & 0x00000000FFFFFFFFllu;
  return (uint32_t)w;
}

std::tuple<bool, int32_t, int32_t> ESPZoneDetect::FindPolygon(const uint32_t wantedId) const {
  uint32_t polygonId { 0 },
           bboxIndex { m_bboxOffset },
           metadataIndex { 0 },
           polygonIndex { 0 };

  while (bboxIndex < m_metadataOffset) {
    uint64_t polygonIndexDelta;
    int32_t metadataIndexDelta;
    int32_t tmp;
    if (!DecodeVariableLengthSigned(bboxIndex, tmp)) break;
    if (!DecodeVariableLengthSigned(bboxIndex, tmp)) break;
    if (!DecodeVariableLengthSigned(bboxIndex, tmp)) break;
    if (!DecodeVariableLengthSigned(bboxIndex, tmp)) break;
    if (!DecodeVariableLengthSigned(bboxIndex, metadataIndexDelta))
      break;
    if (!DecodeVariableLengthUnsigned(bboxIndex, polygonIndexDelta))
      break;

    metadataIndex += metadataIndexDelta;
    polygonIndex += (uint32_t)polygonIndexDelta;

    if (polygonId == wantedId) {
      return {
        true,
        metadataIndex + m_metadataOffset,
        polygonIndex + m_dataOffset
      };
    }

    polygonId++;
  }

  return { false, 0, 0 };
}

std::vector<int32_t> ESPZoneDetect::PolygonToListInternal(const uint32_t polygonIndex) const {
  std::unique_ptr<Reader> reader {new Reader { this, polygonIndex }};

  std::vector<int32_t> list;
  bool loop { true };
  while (loop) {
    switch (auto [result, pointLat, pointLon]{reader->GetPoint()}; result) {
      case Reader::PointOK:
        if (list.size() < 1048576) {
          list.push_back(pointLat);
          list.push_back(pointLon);
          break;
        }
      case Reader::PointError:
        list.clear();
      case Reader::PointDone:
        loop = false;
        break;
    }
  }

  return list;
}

std::vector<double> ESPZoneDetect::PolygonToList(const uint32_t polygonId) const {
  std::vector<double> flData;

  auto [found, metaDataIndex, polygonIndex] {FindPolygon(polygonId)};
  if (!found) { return flData;
  }

  const auto data = PolygonToListInternal(polygonIndex);

  flData.reserve(data.size());
  if (data.empty() || flData.capacity() < data.size()) {
    return flData;
  }

  for (auto coord = data.begin(); coord < data.end(); advance(coord, 2)) {
    flData.push_back(FixedPointToDouble(*coord, 90)); // lat
    flData.push_back(FixedPointToDouble(*coord + 1, 180)); // lon
  }

  return flData;
}

std::tuple<ESPZoneDetect::ZDLookupResult, uint64_t>
  ESPZoneDetect::PointInPolygon(const uint32_t polygonIndex, const int32_t latFixedPoint,
                                const int32_t lonFixedPoint) const {
  int32_t prevLat { 0 }, prevLon { 0 },
          prevQuadrant { 0 }, winding { 0 };
  bool first { true };
  uint64_t distanceSqrMin;

  std::unique_ptr<Reader> reader{new Reader{this, polygonIndex}};

  while (true) {
    auto [result, pointLat, pointLon] {reader->GetPoint()};
    switch (result) {
      case Reader::PointError:
        return {ZD_LOOKUP_PARSE_ERROR, 0};
      case Reader::PointDone:
        switch (winding) {
          case -4:
            return {ZD_LOOKUP_IN_ZONE, distanceSqrMin};
          case 0:
            return {ZD_LOOKUP_NOT_IN_ZONE, distanceSqrMin};
          case 4:
            return {ZD_LOOKUP_IN_EXCLUDED_ZONE, distanceSqrMin};
          default:
            break;
        }
        /* Should not happen */
        return {ZD_LOOKUP_ON_BORDER_SEGMENT, 0};
      default:
        break;
    }

    /* Check if point is ON the border */
    if (pointLat == latFixedPoint && pointLon == lonFixedPoint) {
      return { ZD_LOOKUP_ON_BORDER_VERTEX, 0 };
    }

    /* Find quadrant */
    int quadrant;
    if (pointLat >= latFixedPoint) {
      if (pointLon >= lonFixedPoint) {
        quadrant = 0;
      } else {
        quadrant = 1;
      }
    } else {
      if (pointLon >= lonFixedPoint) {
        quadrant = 3;
      } else {
        quadrant = 2;
      }
    }

    if (!first) {
      int windingNeedCompare = 0, lineIsStraight = 0;
      double a { 0 }, b { 0 };

      /* Calculate winding number */
      if (quadrant == prevQuadrant) {
        /* Do nothing */
      } else if (quadrant == (prevQuadrant + 1) % 4) {
        winding++;
      } else if ((quadrant + 1) % 4 == prevQuadrant) {
        winding--;
      } else {
        windingNeedCompare = 1;
      }

      /* Avoid horizontal and vertical lines */
      if ((pointLon == prevLon || pointLat == prevLat)) {
        lineIsStraight = 1;
      }

      /* Calculate the parameters of y=ax+b if needed */
      if (!lineIsStraight && (distanceSqrMin || windingNeedCompare)) {
        a = ((double)pointLat - (double)prevLat) /
            ((double)pointLon - (double)prevLon);
        b = (double)pointLat - a * (double)pointLon;
      }

      auto onStraight { PointInBox(pointLat, latFixedPoint, prevLat, pointLon,
                                   lonFixedPoint, prevLon) };
      if (lineIsStraight && (onStraight || windingNeedCompare)) {
        return { ZD_LOOKUP_ON_BORDER_SEGMENT, 0 };
      }

      /* Jumped two quadrants. */
      if (windingNeedCompare) {
        /* Check if the target is on the border */
        const int32_t intersectLon = (int32_t)(((double)latFixedPoint - b) / a);
        if (intersectLon >= lonFixedPoint - 1 &&
            intersectLon <= lonFixedPoint + 1) {
          return { ZD_LOOKUP_ON_BORDER_SEGMENT, 0 };
        }

        /* Ok, it's not. In which direction did we go round the target? */
        const int sign = (intersectLon < lonFixedPoint) ? 2 : -2;
        if (quadrant == 2 || quadrant == 3) {
          winding += sign;
        } else {
          winding -= sign;
        }
      }

      /* Calculate closest point on line (if needed) */
      {
        double closestLon, closestLat;
        if (!lineIsStraight) {
          closestLon =
              ((double)lonFixedPoint + a * (double)latFixedPoint - a * b) /
              (a * a + 1);
          closestLat =
              (a * ((double)lonFixedPoint + a * (double)latFixedPoint) + b) /
              (a * a + 1);
        } else {
          if (pointLon == prevLon) {
            closestLon = (double)pointLon;
            closestLat = (double)latFixedPoint;
          } else {
            closestLon = (double)lonFixedPoint;
            closestLat = (double)pointLat;
          }
        }

        const auto closestInBox
            { PointInBox(pointLon, (int32_t)closestLon, prevLon, pointLat,
                         (int32_t)closestLat, prevLat) };

        int64_t diffLat, diffLon;
        if (closestInBox) {
          /* Calculate squared distance to segment. */
          diffLat = (int64_t)(closestLat - (double)latFixedPoint);
          diffLon = (int64_t)(closestLon - (double)lonFixedPoint);
        } else {
          /*
           * Calculate squared distance to vertices
           * It is enough to check the current point since the polygon is
           * closed.
           */
          diffLat = (int64_t)(pointLat - latFixedPoint);
          diffLon = (int64_t)(pointLon - lonFixedPoint);
        }

        /* Note: lon has half scale */
        uint64_t distanceSqr =
            (uint64_t)(diffLat * diffLat) + (uint64_t)(diffLon * diffLon) * 4;
        if (distanceSqr < distanceSqrMin) distanceSqrMin = distanceSqr;
      }
    }

    prevQuadrant = quadrant;
    prevLat = pointLat;
    prevLon = pointLon;
    first = false;
  };
}

// ---- Reader ----
ESPZoneDetect::Reader::Reader(const ESPZoneDetect *parent, uint32_t polygonIndex)
    : m_parent(parent), m_polygonIndex(polygonIndex) {}

ESPZoneDetect::Reader::~Reader(){}

std::tuple<ESPZoneDetect::Reader::GetPointResult, int32_t, int32_t>
ESPZoneDetect::Reader::GetPoint() {
  int32_t diffLat{0}, diffLon{0};

  while (true) {
    if (m_done > 1) {
      return { PointDone, m_pointLat, m_pointLon };
    }

    bool referenceDone { false };
    uint64_t point { 0 };

    if (!m_referenceDirection) {
      if (!m_parent->DecodeVariableLengthUnsigned(m_polygonIndex, point))
        return {PointError, 0, 0};
    } else {
      if (m_referenceDirection > 0) {
        /* Read reference forward */
        if (!m_parent->DecodeVariableLengthUnsigned(m_referenceStart, point))
          return {PointError, 0, 0};
        if (m_referenceStart >= m_referenceEnd) {
          referenceDone = true;
        }
      } else if (m_referenceDirection < 0) {
        /* Read reference backwards */
        if (!m_parent->DecodeVariableLengthUnsignedReverse(m_referenceStart,
                                                           point))
          return {PointError, 0, 0};
        if (m_referenceStart <= m_referenceEnd) {
          referenceDone = true;
        }
      }
    }

    if (!point) {
      /* This is a special marker, it is not allowed in reference mode */
      if (m_referenceDirection) {
        return {PointError, 0, 0};
      }

      uint64_t value;
      if (!m_parent->DecodeVariableLengthUnsigned(m_polygonIndex, value))
        return {PointError, 0, 0};

      switch (value) {
      case 0:
        m_done = 2;
        break;
      case 1: {
        int32_t diff;
        uint64_t start;
        if (!m_parent->DecodeVariableLengthUnsigned(m_polygonIndex, start))
          return {PointError, 0, 0};
        if (!m_parent->DecodeVariableLengthSigned(m_polygonIndex, diff))
          return {PointError, 0, 0};

        m_referenceStart = m_parent->m_dataOffset + (uint32_t)start;
        m_referenceEnd = m_parent->m_dataOffset + (uint32_t)(start + diff);
        m_referenceDirection = diff;
        if (diff < 0) {
          m_referenceStart--;
          m_referenceEnd--;
        }
        continue;
      }
      default:
        break;
      }
    } else {
      diffLat =
          (int32_t)m_parent->DecodeUnsignedToSigned(m_parent->Unshuffle(point)),
      diffLon =
          (int32_t)m_parent->DecodeUnsignedToSigned(m_parent->Unshuffle(point >> 1));
      if (m_referenceDirection < 0) {
        diffLat = -diffLat;
        diffLon = -diffLon;
      }
    }

    if (!m_done) {
      m_pointLat += diffLat;
      m_pointLon += diffLon;
      if (m_first) {
        m_firstLat = m_pointLat;
        m_firstLon = m_pointLon;
      }
    } else {
      /* Close the polygon (the closing point is not encoded) */
      m_pointLat = m_firstLat;
      m_pointLon = m_firstLon;
      m_done = 2;
    }

    m_first = false;

    if (referenceDone) { m_referenceDirection = 0; }

    return { PointOK, m_pointLat, m_pointLon };
  }
}
