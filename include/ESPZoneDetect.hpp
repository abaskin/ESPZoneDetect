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

/*
  The low accuracy database is 1MB (timezone16.bin, ~0.5km) and the higher
  accuracy one 4MB (timezone21.bin, ~20m).

  The number indicates the resolution. The 21 file has a higher resolution for
  storing the borders, but it is larger. The 16 file has a longitude resolution of
  0.0055 degrees (~0.5km) and the 21 file has 0.00017 degrees (~20m).

  The safezone result indicates how close you are to the nearest border (flat
  earth model using lat and lon as x and y), so you can know when to do a new
  lookup.
*/

#include <Arduino.h>
#include <FS.h>
#include <stdint.h>

#include <array>
#include <functional>
#include <memory>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#ifndef INCL_ESPZONEDETECT_H_
#define INCL_ESPZONEDETECT_H_

class ESPZoneDetect{
  public:
    using ZDLookupResult = enum {
      ZD_LOOKUP_IGNORE = -3,
      ZD_LOOKUP_END = -2,
      ZD_LOOKUP_PARSE_ERROR = -1,
      ZD_LOOKUP_NOT_IN_ZONE = 0,
      ZD_LOOKUP_IN_ZONE = 1,
      ZD_LOOKUP_IN_EXCLUDED_ZONE = 2,
      ZD_LOOKUP_ON_BORDER_VERTEX = 3,
      ZD_LOOKUP_ON_BORDER_SEGMENT = 4,
      ZD_LOOKUP_PARSE_OK = 5
    };

    using ZDInternalError = enum {
      ZD_E_DB_OPEN = 0,
      ZD_E_DB_SIZE,
      ZD_E_DB_HUGE,
      ZD_E_DB_SEEK,
      ZD_E_DB_MMAP,
      ZD_E_DB_MUNMAP,
      ZD_E_DB_CLOSE,
      ZD_E_PARSE_HEADER
    };

    using ZoneDetectResult = struct {
      ZDLookupResult lookupResult;
      uint32_t polygonId,
               metaId;
      std::unordered_map<std::string, std::string> fields;
    };

    using zdErrorHandler_t = std::function<void(ZDInternalError, int32_t)>;
    using zdCleanup_t = std::function<void(void)>;

    ESPZoneDetect();
    ~ESPZoneDetect();

    bool OpenDatabaseFromMemory(void* buffer, const size_t length);
    bool OpenDatabase(fs::File* fd);
    void SetErrorHandler(zdErrorHandler_t handler);
    void SetCleanUp(zdCleanup_t cleanUp);

    std::pair<std::vector<ZoneDetectResult>, double> // safezone
        Lookup(const double lat, const double lon) const;
    std::string LookupName(const double lat, const double lon) const;
    std::string LookupPosix(const double lat, const double lon) const;
    std::array<std::string, 2> LookupBoth(const double lat, const double lon) const;

    const char* LookupResultToString(const ZDLookupResult result) const;
    const char* GetErrorString(const ZDInternalError errZD) const;

    const char* GetNotice() const;
    uint8_t GetTableType() const;

   private:
    class mmapRO {
      public:
        mmapRO(){}
        ~mmapRO(){}
        void setFile(fs::File* fp) {
          m_fp = fp;
          m_fileBuffer.reset(new uint8_t[m_bufferSize]);
          m_filePosition = m_fp->size() + 1;
        }

        void setMemory(uint8_t* buffer) {
          m_memBuffer = buffer;
        }

        uint8_t* at(const uint32_t index) const {
          if (!m_fileBuffer) { return &m_memBuffer[index]; }
          if (index < m_filePosition ||
              index > m_filePosition + m_bufferSize - sizeof(uint64_t) - 1) {
            m_filePosition = index;
            m_fp->seek(m_filePosition, SeekSet);
            m_fp->read(m_fileBuffer.get(), m_bufferSize);
          }
          return &m_fileBuffer[index - m_filePosition];
        }

        uint64_t* atUint64(const uint32_t index) const {
          std::memcpy(m_uint64Buffer.get(), at(index), sizeof(uint64_t));
          return m_uint64Buffer.get();
        }

       private:
        const size_t m_bufferSize { 1024 };
        fs::File* m_fp { nullptr };
        std::unique_ptr<uint8_t[]> m_fileBuffer;
        std::unique_ptr<uint64_t> m_uint64Buffer {new uint64_t};
        uint8_t* m_memBuffer;
        mutable uint32_t m_filePosition;
    };

    class Reader {
      public:
        using GetPointResult = enum {
          PointDone = 0,
          PointError,
          PointOK,
        };

        Reader(const ESPZoneDetect* parent, const uint32_t polygonIndex);
        ~Reader();

        std::tuple<GetPointResult, int32_t, int32_t> GetPoint();

       private:
        const ESPZoneDetect* m_parent;
        uint32_t m_polygonIndex;

        bool m_first {true};
        uint8_t m_done {0};
        uint32_t m_referenceStart{0}, m_referenceEnd{0};
        int32_t m_referenceDirection{0};

        int32_t m_pointLat{0}, m_pointLon{0};
        int32_t m_firstLat{0}, m_firstLon{0};
    };

    int32_t DoubleToFixedPoint(const double input, const double scale) const;
    double FixedPointToDouble(const int32_t input, const double scale) const;

    bool DecodeVariableLengthUnsigned(uint32_t& index, uint64_t& result) const ;
    bool DecodeVariableLengthUnsignedReverse(uint32_t& index, uint64_t& result) const ;
    bool DecodeVariableLengthSigned(uint32_t& index, int32_t& result) const;
    int64_t DecodeUnsignedToSigned(const uint64_t value) const;

    std::string ParseString(uint32_t& index) const;
    ZDLookupResult ParseHeader();

    bool PointInBox(const int32_t xl, const int32_t x, const int32_t xr,
                    const int32_t yl, const int32_t y, const int32_t yr) const;
    uint32_t Unshuffle(uint64_t w) const;

    std::tuple<bool, int32_t, int32_t> FindPolygon(const uint32_t wantedId) const;
    std::vector<int32_t> PolygonToListInternal(const uint32_t polygonIndex) const;
    std::vector<double> PolygonToList(const uint32_t polygonId) const;
    std::tuple<ZDLookupResult, uint64_t> PointInPolygon(
      const uint32_t polygonIndex, const int32_t latFixedPoint, const int32_t lonFixedPoint) const;

    std::string getPosix(const std::string& tzName) const;

    zdErrorHandler_t m_zdErrorHandler{[](ZDInternalError, int32_t){}};
    zdCleanup_t m_cleanUp{[](){}};

    uint32_t m_length{0},
             m_bboxOffset{0},
             m_metadataOffset{0},
             m_dataOffset{0};

    uint8_t m_tableType{0},
            m_precision{0};

    std::unique_ptr<mmapRO> m_mapping{new mmapRO};

    std::string m_notice{""};
    std::vector<std::string> m_fieldNames;
};

#endif // INCL_ESPZONEDETECT_H_
