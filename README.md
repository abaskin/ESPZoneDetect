# ESP GPS TimeZone

This is a library to determine the timezone name for a given GPS coordinate. The
library is a refactor of [ZoneDetect](https://github.com/BertoldVdb/ZoneDetect)
for C++ and ESP8266/ESP32.

## Using the library

An ESPZoneDetect object is first created and the database to use (either a file
or memory buffer) is specified. Lookup or LookupString can then be used to get
the timezone or country name for a pair of coordinates.

## Data Files

The data files used are from [ZoneDetect](https://github.com/BertoldVdb/ZoneDetect).
It may be required to increase the size of the filesystem based on the data
file used. Depending on the data file used either the country or timezone name
can be determined. Only v1 database files are supported.

## Declaring the ESPZoneDetect object

```c++
ESPZoneDetect objectName();
```

## Object Methods

```c++
bool OpenDatabaseFromMemory(void* buffer, size_t length)
```

- **buffer** - the address of the in memory database
- **length** - the length of the in memory database

Open a database stored in memory.

```c++
bool OpenDatabase(fs::File* fd)
```

- **fd** - the address of the file descriptor for the database in the filesystem

Open a database stored in a file.

```c++
void SetErrorHandler(zdErrorHandler_t handler)
```

- **handler** - A handler for errors encountered looking up the timezone/country
data.

Set an error handler.

```c++
void SetCleanUp(zdCleanup_t cleanUp)
```

- **cleanUp** - A handler for cleanup when the ESPZoneDetect object is destroyed.

Set a handler to be called on object destruction. This can be used to close the
database file or release the database memory.

```c++
std::tuple<std::vector<ZoneDetectResult>, double> Lookup(double lat, double lon)
```

- **lat** - the latitude of the location to lookup
- **lon** - the longitude of the location to lookup

Lookup the given GPS coordinates and return timezone or country information. The
double value in the result is the distance to the nearest border and may be used
for further processing depending in the accuracy of the databased used.

```c++
std::string LookupString(double lat, double lon)
```

- **lat** - the latitude of the location to lookup
- **lon** - the longitude of the location to lookup

Lookup the given GPS coordinates and return timezone or country name as a string.


```c++
LookupResultToString(ZDLookupResult result)
```

- **result** - the lookup result

Return the result string for the ZDLookupResult found in the ZoneDetectResult
structure.

```c++
const char* GetErrorString(ZDInternalError errZD)
```

- **errZD** - the error value

Return the error string for a ZDInternalError value. This is mostly used in the
error handler.
