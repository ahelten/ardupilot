/*
 * Location.cpp
 */

#include "Location.h"

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Terrain/AP_Terrain.h>
#include <stdio.h>

static const int64_t DEGS_90e9 = 90e9;
static const int64_t DEGS_180e9 = 180e9;
static const int64_t DEGS_360e9 = 360e9;

AP_Terrain *Location::_terrain = nullptr;

/// constructors
Location::Location()
{
    zero();
}

const Location definitely_zero{};
bool Location::is_zero(void) const
{
    return !memcmp(this, &definitely_zero, sizeof(*this));
}

void Location::zero(void)
{
    memset(this, 0, sizeof(*this));
}

// Construct location using position (NEU) from ekf_origin for the given altitude frame
Location::Location(int32_t latitude, int32_t longitude, int32_t alt_in_cm, AltFrame frame)
{
    zero();
    lat = latitude;
    lng = longitude;
    set_alt_cm(alt_in_cm, frame);
}

Location::Location(int32_t latitude, int32_t longitude, int8_t latitude_hp, int8_t longitude_hp, int32_t alt_in_cm, AltFrame frame)
{
  zero();
  lat = latitude;
  lng = longitude;
  // INCLUDE_HIGH_PRECISION_GPS -- might as well leave these enabled since the "_hp" fields are enabled
  lat_hp = latitude_hp;
  lng_hp = longitude_hp;
  set_alt_cm(alt_in_cm, frame);
}

Location::Location(const Vector3f &ekf_offset_neu, AltFrame frame)
{
    zero();

    // store alt and alt frame
    set_alt_cm(ekf_offset_neu.z, frame);

    // calculate lat, lon
    Location ekf_origin;
    if (AP::ahrs().get_origin(ekf_origin)) {
        lat = ekf_origin.lat;
        lng = ekf_origin.lng;
        // INCLUDE_HIGH_PRECISION_GPS -- might as well leave these enabled since the "_hp" fields are enabled
        lat_hp = ekf_origin.lat_hp;
        lng_hp = ekf_origin.lng_hp;

        offset(ekf_offset_neu.x * 0.01, ekf_offset_neu.y * 0.01);
    }
}

Location::Location(const Vector3d &ekf_offset_neu, AltFrame frame)
{
    zero();

    // store alt and alt frame
    set_alt_cm(ekf_offset_neu.z, frame);

    // calculate lat, lon
    Location ekf_origin;
    if (AP::ahrs().get_origin(ekf_origin)) {
        lat = ekf_origin.lat;
        lng = ekf_origin.lng;
        // INCLUDE_HIGH_PRECISION_GPS -- might as well leave these enabled since the "_hp" fields are enabled
        lat_hp = ekf_origin.lat_hp;
        lng_hp = ekf_origin.lng_hp;

        offset(ekf_offset_neu.x * 0.01, ekf_offset_neu.y * 0.01);
    }
}

void Location::update_from_radians(double lat_rad, double lng_rad)
{
    // We use '* 1e9' because we want "high-precision" lat/lng when it's available
    int64_t hplat = lat_rad * RAD_TO_DEG_DOUBLE * 1e9;
    int64_t hplng = lng_rad * RAD_TO_DEG_DOUBLE * 1e9;

    set_highprecision(hplat, hplng);
}

void Location::set_alt_cm(int32_t alt_cm, AltFrame frame)
{
    alt = alt_cm;
    relative_alt = false;
    terrain_alt = false;
    origin_alt = false;
    switch (frame) {
        case AltFrame::ABSOLUTE:
            // do nothing
            break;
        case AltFrame::ABOVE_HOME:
            relative_alt = true;
            break;
        case AltFrame::ABOVE_ORIGIN:
            origin_alt = true;
            break;
        case AltFrame::ABOVE_TERRAIN:
            // we mark it as a relative altitude, as it doesn't have
            // home alt added
            relative_alt = true;
            terrain_alt = true;
            break;
    }
}

// converts altitude to new frame
bool Location::change_alt_frame(AltFrame desired_frame)
{
    int32_t new_alt_cm;
    if (!get_alt_cm(desired_frame, new_alt_cm)) {
        return false;
    }
    set_alt_cm(new_alt_cm, desired_frame);
    return true;
}

// get altitude frame
Location::AltFrame Location::get_alt_frame() const
{
    if (terrain_alt) {
        return AltFrame::ABOVE_TERRAIN;
    }
    if (origin_alt) {
        return AltFrame::ABOVE_ORIGIN;
    }
    if (relative_alt) {
        return AltFrame::ABOVE_HOME;
    }
    return AltFrame::ABSOLUTE;
}

/// get altitude in desired frame
bool Location::get_alt_cm(AltFrame desired_frame, int32_t &ret_alt_cm) const
{
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (!initialised()) {
        AP_HAL::panic("Should not be called on invalid location: Location cannot be (0, 0, 0)");
    }
#endif
    Location::AltFrame frame = get_alt_frame();

    // shortcut if desired and underlying frame are the same
    if (desired_frame == frame) {
        ret_alt_cm = alt;
        return true;
    }

    // check for terrain altitude
    float alt_terr_cm = 0;
    if (frame == AltFrame::ABOVE_TERRAIN || desired_frame == AltFrame::ABOVE_TERRAIN) {
#if AP_TERRAIN_AVAILABLE
        if (_terrain == nullptr || !_terrain->height_amsl(*this, alt_terr_cm, true)) {
            return false;
        }
        // convert terrain alt to cm
        alt_terr_cm *= 100.0f;
#else
        return false;
#endif
    }

    // convert alt to absolute
    int32_t alt_abs = 0;
    switch (frame) {
        case AltFrame::ABSOLUTE:
            alt_abs = alt;
            break;
        case AltFrame::ABOVE_HOME:
            if (!AP::ahrs().home_is_set()) {
                return false;
            }
            alt_abs = alt + AP::ahrs().get_home().alt;
            break;
        case AltFrame::ABOVE_ORIGIN:
            {
                // fail if we cannot get ekf origin
                Location ekf_origin;
                if (!AP::ahrs().get_origin(ekf_origin)) {
                    return false;
                }
                alt_abs = alt + ekf_origin.alt;
            }
            break;
        case AltFrame::ABOVE_TERRAIN:
            alt_abs = alt + alt_terr_cm;
            break;
    }

    // convert absolute to desired frame
    switch (desired_frame) {
        case AltFrame::ABSOLUTE:
            ret_alt_cm = alt_abs;
            return true;
        case AltFrame::ABOVE_HOME:
            if (!AP::ahrs().home_is_set()) {
                return false;
            }
            ret_alt_cm = alt_abs - AP::ahrs().get_home().alt;
            return true;
        case AltFrame::ABOVE_ORIGIN:
            {
                // fail if we cannot get ekf origin
                Location ekf_origin;
                if (!AP::ahrs().get_origin(ekf_origin)) {
                    return false;
                }
                ret_alt_cm = alt_abs - ekf_origin.alt;
                return true;
            }
        case AltFrame::ABOVE_TERRAIN:
            ret_alt_cm = alt_abs - alt_terr_cm;
            return true;
    }
    return false;  // LCOV_EXCL_LINE  - not reachable
}

bool Location::get_vector_xy_from_origin_NE(Vector2f &vec_ne) const
{
    Location ekf_origin;
    if (!AP::ahrs().get_origin(ekf_origin)) {
        return false;
    }

#ifdef INCLUDE_HIGH_PRECISION_GPS
    vec_ne = get_distance_NE(ekf_origin);
    vec_ne.x *= -100; // invert and convert to cm
    vec_ne.y *= -100; // invert and convert to cm
#else
    vec_ne.x = (lat-ekf_origin.lat) * LATLON_TO_CM;
    vec_ne.y = diff_longitude(lng,ekf_origin.lng) * LATLON_TO_CM * longitude_scale((lat+ekf_origin.lat)/2);
#endif
    return true;
}

bool Location::get_vector_from_origin_NEU(Vector3f &vec_neu) const
{
    // convert lat, lon
    Vector2f vec_ne;
    if (!get_vector_xy_from_origin_NE(vec_ne)) {
        return false;
    }
    vec_neu.x = vec_ne.x;
    vec_neu.y = vec_ne.y;

    // convert altitude
    int32_t alt_above_origin_cm = 0;
    if (!get_alt_cm(AltFrame::ABOVE_ORIGIN, alt_above_origin_cm)) {
        return false;
    }
    vec_neu.z = alt_above_origin_cm;

    return true;
}

// return distance in meters between two locations
ftype Location::get_distance(const struct Location &loc2) const
{
#ifdef INCLUDE_HIGH_PRECISION_GPS
    Vector2f vec_ne = get_distance_NE(loc2);
    return vec_ne.length();
#else
    ftype dlat = (ftype)(loc2.lat - lat);
    ftype dlng = ((ftype)diff_longitude(loc2.lng,lng)) * longitude_scale((lat+loc2.lat)/2);
    return norm(dlat, dlng) * LOCATION_SCALING_FACTOR;
#endif
}


/*
  return the distance in meters in North/East plane as a N/E vector
  from loc1 to loc2
 */
Vector2f Location::get_distance_NE(const Location &loc2) const
{
#ifdef INCLUDE_HIGH_PRECISION_GPS
    /// @todo Update this to use integer math!!!!!!
    const double loc1_lat_hp = (double)lat + (lat_hp / 100.0);
    const double loc1_lng_hp = (double)lng + (lng_hp / 100.0);
    const double loc2_lat_hp = (double)loc2.lat + (loc2.lat_hp / 100.0);
    const double loc2_lng_hp = (double)loc2.lng + (loc2.lng_hp / 100.0);

    double dlat = loc2_lat_hp - loc1_lat_hp;
    double dlng = loc2_lng_hp - loc1_lng_hp;
    if (dlng > 180e7) {
        dlng -= 360e7;
    }
    else if (dlng < -180e7) {
        dlng += 360e7;
    }

    dlat *= double(LOCATION_SCALING_FACTOR);
    dlng *= double(LOCATION_SCALING_FACTOR) * longitude_scale((loc2.lat+lat)/2);

    return Vector2f(float(dlat), float(dlng));
#else
  return Vector2f((loc2.lat - lat) * LOCATION_SCALING_FACTOR,
                  diff_longitude(loc2.lng,lng) * LOCATION_SCALING_FACTOR * longitude_scale((loc2.lat+lat)/2));
#endif
}

// return the distance in meters in North/East/Down plane as a N/E/D vector to loc2
Vector3f Location::get_distance_NED(const Location &loc2) const
{
#ifdef INCLUDE_HIGH_PRECISION_GPS
    Vector2f vec_ne = get_distance_NE(loc2);
    return Vector3f(vec_ne.x, vec_ne.y, (alt - loc2.alt) * 0.01);
#else
    return Vector3f((loc2.lat - lat) * LOCATION_SCALING_FACTOR,
                    diff_longitude(loc2.lng,lng) * LOCATION_SCALING_FACTOR * longitude_scale((lat+loc2.lat)/2),
                    (alt - loc2.alt) * 0.01);
#endif
}

// return the distance in meters in North/East/Down plane as a N/E/D vector to loc2
Vector3d Location::get_distance_NED_double(const Location &loc2) const
{
#ifdef INCLUDE_HIGH_PRECISION_GPS
    Vector2d vec_ne = get_distance_NE_double(loc2);
    return Vector3d(vec_ne.x, vec_ne.y, (alt - loc2.alt) * 0.01);
#else
    return Vector3d((loc2.lat - lat) * double(LOCATION_SCALING_FACTOR),
                    diff_longitude(loc2.lng,lng) * LOCATION_SCALING_FACTOR * longitude_scale((lat+loc2.lat)/2),
                    (alt - loc2.alt) * 0.01);
#endif
}

Vector2d Location::get_distance_NE_double(const Location &loc2) const
{
#ifdef INCLUDE_HIGH_PRECISION_GPS
    /// @todo Update this to use integer math!!!!!!
    const double loc1_lat_hp = (double)lat + (lat_hp / 100.0);
    const double loc1_lng_hp = (double)lng + (lng_hp / 100.0);
    const double loc2_lat_hp = (double)loc2.lat + (loc2.lat_hp / 100.0);
    const double loc2_lng_hp = (double)loc2.lng + (loc2.lng_hp / 100.0);

    double dlat = loc2_lat_hp - loc1_lat_hp;
    double dlng = loc2_lng_hp - loc1_lng_hp;
    if (dlng > 180e7) {
        dlng -= 360e7;
    }
    else if (dlng < -180e7) {
        dlng += 360e7;
    }

    dlat *= double(LOCATION_SCALING_FACTOR);
    dlng *= double(LOCATION_SCALING_FACTOR) * longitude_scale((loc2.lat+lat)/2);

    return Vector2d(dlat, dlng);
#else
    return Vector2d((loc2.lat - lat) * double(LOCATION_SCALING_FACTOR),
                    diff_longitude(loc2.lng,lng) * double(LOCATION_SCALING_FACTOR) * longitude_scale((lat+loc2.lat)/2));
#endif
}

Vector2F Location::get_distance_NE_ftype(const Location &loc2) const
{
#ifdef INCLUDE_HIGH_PRECISION_GPS
    /// @todo Update this to use integer math!!!!!!
    const double loc1_lat_hp = (double)lat + (double)(lat_hp / 100.0);
    const double loc1_lng_hp = (double)lng + (double)(lng_hp / 100.0);
    const double loc2_lat_hp = (double)loc2.lat + (double)(loc2.lat_hp / 100.0);
    const double loc2_lng_hp = (double)loc2.lng + (double)(loc2.lng_hp / 100.0);

    double dlat = loc2_lat_hp - loc1_lat_hp;
    double dlng = loc2_lng_hp - loc1_lng_hp;
    if (dlng > 180e7) {
        dlng -= 360e7;
    }
    else if (dlng < -180e7) {
        dlng += 360e7;
    }

    dlat *= double(LOCATION_SCALING_FACTOR);
    dlng *= double(LOCATION_SCALING_FACTOR) * longitude_scale((loc2.lat+lat)/2);

    return Vector2F(ftype(dlat), ftype(dlng));
#else
    return Vector2F((loc2.lat - lat) * ftype(LOCATION_SCALING_FACTOR),
                    diff_longitude(loc2.lng,lng) * ftype(LOCATION_SCALING_FACTOR) * longitude_scale((lat+loc2.lat)/2));
#endif
}

#ifndef INCLUDE_HIGH_PRECISION_GPS
// extrapolate latitude/longitude given distances (in meters) north and east
void Location::offset_latlng(int32_t &lat, int32_t &lng, ftype ofs_north, ftype ofs_east)
{
    const int32_t dlat = ofs_north * LOCATION_SCALING_FACTOR_INV;
    const int64_t dlng = (ofs_east * LOCATION_SCALING_FACTOR_INV) / longitude_scale(lat+dlat/2);
    lat += dlat;
    lat = limit_lattitude(lat);
    lng = wrap_longitude(dlng+lng);
}
#endif

// extrapolate latitude/longitude given distances (in meters) north and east
void Location::offset(ftype ofs_north, ftype ofs_east)
{
#ifdef INCLUDE_HIGH_PRECISION_GPS
    const double dlat = ofs_north * LOCATION_SCALING_FACTOR_INV;
    const double dlng = (ofs_east * LOCATION_SCALING_FACTOR_INV) / longitude_scale(lat+dlat/2);
    int64_t hplat = ((int64_t)lat * 100) + lat_hp;
    int64_t hplng = ((int64_t)lng * 100) + lng_hp;

    hplat += lround(dlat * 100);
    hplng += lround(dlng * 100);

    set_highprecision(hplat, hplng);
#else
    offset_latlng(lat, lng, ofs_north, ofs_east);
#endif
}

/*
 *  extrapolate latitude/longitude given bearing and distance
 * Note that this function is accurate to about 1mm at a distance of
 * 100m. This function has the advantage that it works in relative
 * positions, so it keeps the accuracy even when dealing with small
 * distances and floating point numbers
 */
void Location::offset_bearing(ftype bearing_deg, ftype distance)
{
    const ftype ofs_north = cosF(radians(bearing_deg)) * distance;
    const ftype ofs_east  = sinF(radians(bearing_deg)) * distance;
    offset(ofs_north, ofs_east);
}

// extrapolate latitude/longitude given bearing, pitch and distance
void Location::offset_bearing_and_pitch(ftype bearing_deg, ftype pitch_deg, ftype distance)
{
    const ftype ofs_north =  cosF(radians(pitch_deg)) * cosF(radians(bearing_deg)) * distance;
    const ftype ofs_east  =  cosF(radians(pitch_deg)) * sinF(radians(bearing_deg)) * distance;
    offset(ofs_north, ofs_east);
    const int32_t dalt =  sinf(radians(pitch_deg)) * distance *100.0f;
    alt += dalt; 
}


ftype Location::longitude_scale(int32_t lat)
{
    ftype scale = cosF(lat * (1.0e-7 * DEG_TO_RAD));
    return MAX(scale, 0.01);
}

/*
 * convert invalid waypoint with useful data. return true if location changed
 */
bool Location::sanitize(const Location &defaultLoc)
{
    bool has_changed = false;
    // convert lat/lng=0 to mean current point
    if (lat == 0 && lng == 0) {
        lat = defaultLoc.lat;
        lng = defaultLoc.lng;
        has_changed = true;
    }

    // convert relative alt=0 to mean current alt
    if (alt == 0 && relative_alt) {
        relative_alt = false;
        alt = defaultLoc.alt;
        has_changed = true;
    }

    // limit lat/lng to appropriate ranges
    if (!check_latlng()) {
        lat = defaultLoc.lat;
        lng = defaultLoc.lng;
        has_changed = true;
    }

    return has_changed;
}

#ifdef INCLUDE_HIGH_PRECISION_GPS
// AMH: A resize of Location is no longer necessary after moving lat_hp/lng_hp to the padding
//      between the Location bit flags and the int32_t alt/lat/lng fields. Now the size remains
//      16 even with the two new fields.
// -- Comment in original branch that resized this to 20:
//JO:
// **** NOT SURE OF THE IMPLICATION OF INCREASING THIS SIZE! ****
#endif
// make sure we know what size the Location object is:
assert_storage_size<Location, 16> _assert_storage_size_Location;


// return bearing in centi-degrees from location to loc2
// @amh: Should we update this to support lat_hp/lng_hp? Seems if loc2 is far enough away (how
//       far?), it most likely will not change the bearing enough to matter.
int32_t Location::get_bearing_to(const struct Location &loc2) const
{
    const int32_t off_x = diff_longitude(loc2.lng,lng);
    const int32_t off_y = (loc2.lat - lat) / loc2.longitude_scale((lat+loc2.lat)/2);
    int32_t bearing = 9000 + atan2f(-off_y, off_x) * DEGX100;
    if (bearing < 0) {
        bearing += 36000;
    }
    return bearing;
}

/*
  return true if lat and lng match. Ignores altitude and options
 */
bool Location::same_latlon_as(const Location &loc2) const
{
#ifdef INCLUDE_HIGH_PRECISION_GPS
    // Add some wiggle room for the high-precision component
    return    (lat == loc2.lat) && (lng == loc2.lng)
           && (abs(lat_hp - loc2.lat_hp) < 5) && (abs(lng_hp - loc2.lng_hp) < 5);
#else
    return (lat == loc2.lat) && (lng == loc2.lng);
#endif
}

// return true when lat and lng are within range
bool Location::check_latlng() const
{
    return check_lat(lat) && check_lng(lng);
}

// see if location is past a line perpendicular to
// the line between point1 and point2 and passing through point2.
// If point1 is our previous waypoint and point2 is our target waypoint
// then this function returns true if we have flown past
// the target waypoint
bool Location::past_interval_finish_line(const Location &point1, const Location &point2) const
{
    return this->line_path_proportion(point1, point2) >= 1.0f;
}

/*
  return the proportion we are along the path from point1 to
  point2, along a line parallel to point1<->point2.

  This will be more than 1 if we have passed point2
 */
float Location::line_path_proportion(const Location &point1, const Location &point2) const
{
    const Vector2f vec1 = point1.get_distance_NE(point2);
    const Vector2f vec2 = point1.get_distance_NE(*this);
    const ftype dsquared = sq(vec1.x) + sq(vec1.y);
    if (dsquared < 0.001f) {
        // the two points are very close together
        return 1.0f;
    }
    return (vec1 * vec2) / dsquared;
}

/*
  wrap longitude for -180e7 to 180e7
 */
int32_t Location::wrap_longitude(int64_t lon)
{
    if (lon > 1800000000L) {
        lon = int32_t(lon-3600000000LL);
    } else if (lon < -1800000000L) {
        lon = int32_t(lon+3600000000LL);
    }
    return int32_t(lon);
}

/*
  get lon1-lon2, wrapping at -180e7 to 180e7
 */
int32_t Location::diff_longitude(int32_t lon1, int32_t lon2)
{
    if ((lon1 & 0x80000000) == (lon2 & 0x80000000)) {
        // common case of same sign
        return lon1 - lon2;
    }
    int64_t dlon = int64_t(lon1)-int64_t(lon2);
    if (dlon > 1800000000LL) {
        dlon -= 3600000000LL;
    } else if (dlon < -1800000000LL) {
        dlon += 3600000000LL;
    }
    return int32_t(dlon);
}

/*
  limit lattitude to -90e7 to 90e7
 */
int32_t Location::limit_lattitude(int32_t lat)
{
    if (lat > 900000000L) {
        lat = 1800000000LL - lat;
    } else if (lat < -900000000L) {
        lat = -(1800000000LL + lat);
    }
    return lat;
}

// update altitude and alt-frame base on this location's horizontal position between point1 and point2
// this location's lat,lon is used to calculate the alt of the closest point on the line between point1 and point2
// origin and destination's altitude frames must be the same
// this alt-frame will be updated to match the destination alt frame
void Location::linearly_interpolate_alt(const Location &point1, const Location &point2)
{
    // new target's distance along the original track and then linear interpolate between the original origin and destination altitudes
    set_alt_cm(point1.alt + (point2.alt - point1.alt) * constrain_float(line_path_proportion(point1, point2), 0.0f, 1.0f), point2.get_alt_frame());
}

// Set high precision lat/lng using "floating point 1e9 degrees". For example, 123.123456789
// degrees would be represented as 123123456789 where the last two digits are the "high
// precision" part.
void Location::set_highprecision(int64_t hplat_1e9_degs, int64_t hplng_1e9_degs)
{
    if (hplat_1e9_degs > DEGS_90e9) {
        hplat_1e9_degs -= DEGS_180e9;
    }
    else if (hplat_1e9_degs < -DEGS_90e9) {
        hplat_1e9_degs += DEGS_180e9;
    }
    if (hplng_1e9_degs > DEGS_180e9) {
        hplng_1e9_degs -= DEGS_360e9;
    }
    else if (hplng_1e9_degs < -DEGS_180e9) {
        hplng_1e9_degs += DEGS_360e9;
    }

    // I think we only want the integer math on this since double-precision math was "failing"
    // on pixhawk (it was rounding by quite a bit).
    if (hplat_1e9_degs < 0) {
        lat = (int32_t)((hplat_1e9_degs - 50) / 100);
    } else {
        lat = (int32_t)((hplat_1e9_degs + 50) / 100);
    }
    if (hplng_1e9_degs < 0) {
        lng = (int32_t)((hplng_1e9_degs - 50) / 100);
    } else {
        lng = (int32_t)((hplng_1e9_degs + 50) / 100);
    }

    lat_hp = (int8_t)(hplat_1e9_degs - ((int64_t)lat * 100));
    lng_hp = (int8_t)(hplng_1e9_degs - ((int64_t)lng * 100));
}

