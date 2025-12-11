#include <stdbool.h>
#include <stddef.h>

#ifndef OSRMC_H_
#define OSRMC_H_

/*
 * libosrmc Interface Overview
 * ===========================
 *
 * Workflow
 * --------
 *
 * Services: Nearest, Route, Table, Match, Trip, Tile
 *
 * 1. Create config with base path (or NULL for shared memory via osrm-datastore)
 * 2. Optionally configure algorithm and constraints
 * 3. Construct OSRM instance
 * 4. Create service-specific params, add coordinates, configure options
 * 5. Query service and extract results
 *
 * Example:
 *
 *   osrmc_config_t config = osrmc_config_construct(path, &error);
 *   osrmc_osrm_t osrm = osrmc_osrm_construct(config, &error);
 *
 *
 * Algorithm Selection
 * -------------------
 *
 * Default: CH (Contraction Hierarchies)
 * Alternative: MLD (Multi-Level Dijkstra)
 *
 *   osrmc_config_set_algorithm(config, OSRMC_ALGORITHM_MLD, &error);
 *
 *
 * Service Constraints
 * -------------------
 *
 * Set limits via osrmc_config_set_* functions. Use -1 for unlimited.
 *
 * - max_locations_trip: Trip service locations (default: -1)
 * - max_locations_viaroute: Route service locations (default: -1)
 * - max_locations_distance_table: Table service locations (default: -1)
 * - max_locations_map_matching: Match service locations (default: -1)
 * - max_radius_map_matching: Map matching radius in meters (default: -1.0)
 * - max_results_nearest: Nearest service results (default: -1)
 * - default_radius: Coordinate snapping radius in meters (default: -1.0)
 * - max_alternatives: Alternative routes (default: 3)
 *
 *
 * Querying Services
 * -----------------
 *
 * 1. Create params: osrmc_service_params_construct
 * 2. Add coordinates: osrmc_params_add_coordinate (or osrmc_params_add_coordinate_with)
 * 3. Configure options: service-specific setters
 * 4. Query: osrmc_service(osrm, params, &error)
 * 5. Extract results: osrmc_service_response_* getters
 * 6. Cleanup: osrmc_service_response_destruct
 *
 * Example:
 *
 *   osrmc_route_params_t params = osrmc_route_params_construct(&error);
 *   osrmc_params_add_coordinate((osrmc_params_t)params, lon, lat, &error);
 *   osrmc_route_params_set_alternatives(params, 1, &error);
 *   osrmc_route_response_t response = osrmc_route(osrm, params, &error);
 *   double distance = osrmc_route_response_distance(response, &error);
 *   osrmc_route_response_destruct(response);
 *
 *
 * Types
 * -----
 *
 * Opaque types with osrmc_type_construct/destruct naming convention.
 *
 *
 * Error Handling
 * --------------
 *
 * Functions take osrmc_error_t* out parameter. On failure:
 * - Error object is populated (caller owns it)
 * - Get message via osrmc_error_message
 * - Destruct via osrmc_error_destruct
 *
 * Example:
 *
 *   osrmc_error_t error = NULL;
 *   params = osrmc_route_params_construct(&error);
 *   if (error) {
 *     fprintf(stderr, "Error: %s\n", osrmc_error_message(error));
 *     osrmc_error_destruct(error);
 *     return EXIT_FAILURE;
 *   }
 *
 *
 * Response String Pointers
 * ------------------------
 *
 * Functions returning const char* (name, hint, data_version, etc.) return pointers
 * to internal response data. Valid only while response exists. Do not free.
 * Returns nullptr if field is missing.
 *
 * Example:
 *
 *   const char* name = osrmc_route_response_waypoint_name(response, 0, &error);
 *   if (name) {
 *     printf("Name: %s\n", name);  // Valid while response exists
 *   }
 *   osrmc_route_response_destruct(response);
 *   // name pointer invalid after this point
 *
 */

#ifdef __cplusplus
extern "C" {
#endif

/* ABI Stability */

#if __GNUC__ >= 4
#define OSRMC_API __attribute__((visibility("default")))
#else
#define OSRMC_API
#endif

#define OSRMC_VERSION_MAJOR 6
#define OSRMC_VERSION_MINOR 0
#define OSRMC_VERSION ((OSRMC_VERSION_MAJOR << 16) | OSRMC_VERSION_MINOR)

OSRMC_API unsigned
osrmc_get_version(void);
OSRMC_API int
osrmc_is_abi_compatible(void);

/* Types */
/* Error */
typedef struct osrmc_error* osrmc_error_t;
/* Config*/
typedef struct osrmc_config* osrmc_config_t;

/* OSRM */
typedef struct osrmc_osrm* osrmc_osrm_t;
typedef struct osrmc_params* osrmc_params_t;
typedef struct osrmc_blob* osrmc_blob_t;

/* Nearest */
typedef struct osrmc_nearest_params* osrmc_nearest_params_t;
typedef struct osrmc_nearest_response* osrmc_nearest_response_t;

/* Route */
typedef struct osrmc_route_params* osrmc_route_params_t;
typedef struct osrmc_route_response* osrmc_route_response_t;

/* Table */
typedef struct osrmc_table_params* osrmc_table_params_t;
typedef struct osrmc_table_response* osrmc_table_response_t;

/* Match */
typedef struct osrmc_match_params* osrmc_match_params_t;
typedef struct osrmc_match_response* osrmc_match_response_t;

/* Trip */
typedef struct osrmc_trip_params* osrmc_trip_params_t;
typedef struct osrmc_trip_response* osrmc_trip_response_t;

/* Tile */
typedef struct osrmc_tile_params* osrmc_tile_params_t;
typedef struct osrmc_tile_response* osrmc_tile_response_t;

/* Enums*/

/* Output formats */
/* NOTE: Flatbuffers format is not supported */
typedef enum {
  OSRMC_FORMAT_JSON = 0,
} osrmc_output_format_t;

/* Algorithms */
typedef enum {
  OSRMC_ALGORITHM_CH = 0, /* Contraction Hierarchies (default) */
  OSRMC_ALGORITHM_MLD = 1 /* Multi-Level Dijkstra */
} osrmc_algorithm_t;

/* Snapping */
typedef enum { OSRMC_SNAPPING_DEFAULT = 0, OSRMC_SNAPPING_ANY = 1 } osrmc_snapping_t;

/* Approach */
typedef enum { OSRMC_APPROACH_CURB = 0, OSRMC_APPROACH_UNRESTRICTED = 1, OSRMC_APPROACH_OPPOSITE = 2 } osrmc_approach_t;

/* Geometries */
typedef enum {
  OSRMC_GEOMETRIES_POLYLINE = 0,
  OSRMC_GEOMETRIES_POLYLINE6 = 1,
  OSRMC_GEOMETRIES_GEOJSON = 2
} osrmc_geometries_type_t;

/* Overviews */
typedef enum { OSRMC_OVERVIEW_SIMPLIFIED = 0, OSRMC_OVERVIEW_FULL = 1, OSRMC_OVERVIEW_FALSE = 2 } osrmc_overview_type_t;

/* Annotations */
typedef enum {
  OSRMC_ANNOTATIONS_NONE = 0,
  OSRMC_ANNOTATIONS_DURATION = 1,
  OSRMC_ANNOTATIONS_NODES = 2,
  OSRMC_ANNOTATIONS_DISTANCE = 4,
  OSRMC_ANNOTATIONS_WEIGHT = 8,
  OSRMC_ANNOTATIONS_DATASOURCES = 16,
  OSRMC_ANNOTATIONS_SPEED = 32,
  OSRMC_ANNOTATIONS_ALL = 63
} osrmc_annotations_type_t;

/* Table annotations */
typedef enum {
  OSRMC_TABLE_ANNOTATIONS_NONE = 0,
  OSRMC_TABLE_ANNOTATIONS_DURATION = 1,
  OSRMC_TABLE_ANNOTATIONS_DISTANCE = 2,
  OSRMC_TABLE_ANNOTATIONS_ALL = 3
} osrmc_table_annotations_type_t;

/* Table fallback coordinate*/
typedef enum {
  OSRMC_TABLE_FALLBACK_COORDINATE_INPUT = 0,
  OSRMC_TABLE_FALLBACK_COORDINATE_SNAPPED = 1
} osrmc_table_fallback_coordinate_type_t;

/* Match gaps */
typedef enum { OSRMC_MATCH_GAPS_SPLIT = 0, OSRMC_MATCH_GAPS_IGNORE = 1 } osrmc_match_gaps_type_t;

/* Trip source */
typedef enum { OSRMC_TRIP_SOURCE_ANY = 0, OSRMC_TRIP_SOURCE_FIRST = 1 } osrmc_trip_source_type_t;

/* Trip destination */
typedef enum { OSRMC_TRIP_DESTINATION_ANY = 0, OSRMC_TRIP_DESTINATION_LAST = 1 } osrmc_trip_destination_type_t;


/* Error*/

/* Error code and message getters */
OSRMC_API const char*
osrmc_error_code(osrmc_error_t error);
OSRMC_API const char*
osrmc_error_message(osrmc_error_t error);
/* Error destructor */
OSRMC_API void
osrmc_error_destruct(osrmc_error_t error);

/* Config */

/* Config constructor and destructor */
OSRMC_API osrmc_config_t
osrmc_config_construct(const char* base_path, osrmc_error_t* error);
OSRMC_API void
osrmc_config_destruct(osrmc_config_t config);
/* Config parameter setters */
OSRMC_API void
osrmc_config_set_max_locations_trip(osrmc_config_t config, int max_locations, osrmc_error_t* error);
OSRMC_API void
osrmc_config_set_max_locations_viaroute(osrmc_config_t config, int max_locations, osrmc_error_t* error);
OSRMC_API void
osrmc_config_set_max_locations_distance_table(osrmc_config_t config, int max_locations, osrmc_error_t* error);
OSRMC_API void
osrmc_config_set_max_locations_map_matching(osrmc_config_t config, int max_locations, osrmc_error_t* error);
OSRMC_API void
osrmc_config_set_max_radius_map_matching(osrmc_config_t config, double max_radius, osrmc_error_t* error);
OSRMC_API void
osrmc_config_set_max_results_nearest(osrmc_config_t config, int max_results, osrmc_error_t* error);
OSRMC_API void
osrmc_config_set_default_radius(osrmc_config_t config, double default_radius, osrmc_error_t* error);
OSRMC_API void
osrmc_config_set_max_alternatives(osrmc_config_t config, int max_alternatives, osrmc_error_t* error);
OSRMC_API void
osrmc_config_set_use_shared_memory(osrmc_config_t config, bool use_shared_memory, osrmc_error_t* error);
OSRMC_API void
osrmc_config_set_memory_file(osrmc_config_t config, const char* memory_file, osrmc_error_t* error);
OSRMC_API void
osrmc_config_set_use_mmap(osrmc_config_t config, bool use_mmap, osrmc_error_t* error);
OSRMC_API void
osrmc_config_set_algorithm(osrmc_config_t config, osrmc_algorithm_t algorithm, osrmc_error_t* error);
OSRMC_API void
osrmc_config_disable_feature_dataset(osrmc_config_t config, const char* dataset_name, osrmc_error_t* error);
OSRMC_API void
osrmc_config_set_verbosity(osrmc_config_t config, const char* verbosity, osrmc_error_t* error);
OSRMC_API void
osrmc_config_set_dataset_name(osrmc_config_t config, const char* dataset_name, osrmc_error_t* error);
OSRMC_API void
osrmc_config_clear_disabled_feature_datasets(osrmc_config_t config, osrmc_error_t* error);

/* OSRM */

/* OSRM constructor and destructor */
OSRMC_API osrmc_osrm_t
osrmc_osrm_construct(osrmc_config_t config, osrmc_error_t* error);
OSRMC_API void
osrmc_osrm_destruct(osrmc_osrm_t osrm);
/* OSRM parameter setters*/
OSRMC_API void
osrmc_params_add_coordinate(osrmc_params_t params, double longitude, double latitude, osrmc_error_t* error);
OSRMC_API void
osrmc_params_add_coordinate_with(osrmc_params_t params,
                                 double longitude,
                                 double latitude,
                                 double radius,
                                 int bearing,
                                 int range,
                                 osrmc_error_t* error);
OSRMC_API void
osrmc_params_set_hint(osrmc_params_t params, size_t coordinate_index, const char* hint_base64, osrmc_error_t* error);
OSRMC_API void
osrmc_params_set_radius(osrmc_params_t params, size_t coordinate_index, double radius, osrmc_error_t* error);
OSRMC_API void
osrmc_params_set_bearing(osrmc_params_t params, size_t coordinate_index, int value, int range, osrmc_error_t* error);
OSRMC_API void
osrmc_params_set_approach(osrmc_params_t params,
                          size_t coordinate_index,
                          osrmc_approach_t approach,
                          osrmc_error_t* error);
OSRMC_API void
osrmc_params_add_exclude(osrmc_params_t params, const char* exclude_profile, osrmc_error_t* error);
OSRMC_API void
osrmc_params_set_generate_hints(osrmc_params_t params, int on, osrmc_error_t* error);
OSRMC_API void
osrmc_params_set_skip_waypoints(osrmc_params_t params, int on, osrmc_error_t* error);
OSRMC_API void
osrmc_params_set_snapping(osrmc_params_t params, osrmc_snapping_t snapping, osrmc_error_t* error);
OSRMC_API void
osrmc_params_set_format(osrmc_params_t params, osrmc_output_format_t format, osrmc_error_t* error);
/* OSRM response blob accessors*/
OSRMC_API const char*
osrmc_blob_data(osrmc_blob_t blob);
OSRMC_API size_t
osrmc_blob_size(osrmc_blob_t blob);
OSRMC_API void
osrmc_blob_destruct(osrmc_blob_t blob);

/* Nearest */

/* Nearest parameter constructor and destructor */
OSRMC_API osrmc_nearest_params_t
osrmc_nearest_params_construct(osrmc_error_t* error);
OSRMC_API void
osrmc_nearest_params_destruct(osrmc_nearest_params_t params);
/* Nearest parameter setters*/
OSRMC_API void
osrmc_nearest_params_set_number_of_results(osrmc_nearest_params_t params, unsigned n, osrmc_error_t* error);
/* Nearest response constructor and destructor */
OSRMC_API osrmc_nearest_response_t
osrmc_nearest(osrmc_osrm_t osrm, osrmc_nearest_params_t params, osrmc_error_t* error);
OSRMC_API void
osrmc_nearest_response_destruct(osrmc_nearest_response_t response);
/* Nearest response getters*/
OSRMC_API osrmc_blob_t
osrmc_nearest_response_json(osrmc_nearest_response_t response, osrmc_error_t* error);

/* Route */

/* Route parameter  constructor and destructor*/
OSRMC_API osrmc_route_params_t
osrmc_route_params_construct(osrmc_error_t* error);
OSRMC_API void
osrmc_route_params_destruct(osrmc_route_params_t params);
/* Route parameter setters*/
OSRMC_API void
osrmc_route_params_set_steps(osrmc_route_params_t params, int on, osrmc_error_t* error);
OSRMC_API void
osrmc_route_params_set_alternatives(osrmc_route_params_t params, int on, osrmc_error_t* error);
OSRMC_API void
osrmc_route_params_set_geometries(osrmc_route_params_t params,
                                  osrmc_geometries_type_t geometries,
                                  osrmc_error_t* error);
OSRMC_API void
osrmc_route_params_set_overview(osrmc_route_params_t params, osrmc_overview_type_t overview, osrmc_error_t* error);
OSRMC_API void
osrmc_route_params_set_continue_straight(osrmc_route_params_t params, int on, osrmc_error_t* error);
OSRMC_API void
osrmc_route_params_set_number_of_alternatives(osrmc_route_params_t params, unsigned count, osrmc_error_t* error);
OSRMC_API void
osrmc_route_params_set_annotations(osrmc_route_params_t params,
                                   osrmc_annotations_type_t annotations,
                                   osrmc_error_t* error);
OSRMC_API void
osrmc_route_params_add_waypoint(osrmc_route_params_t params, size_t index, osrmc_error_t* error);
OSRMC_API void
osrmc_route_params_clear_waypoints(osrmc_route_params_t params);
/* Route response constructor and destructor*/
OSRMC_API osrmc_route_response_t
osrmc_route(osrmc_osrm_t osrm, osrmc_route_params_t params, osrmc_error_t* error);
OSRMC_API void
osrmc_route_response_destruct(osrmc_route_response_t response);
/* Route response getters*/
OSRMC_API osrmc_blob_t
osrmc_route_response_json(osrmc_route_response_t response, osrmc_error_t* error);

/* Table */

/* Table parameter constructor and destructor */
OSRMC_API osrmc_table_params_t
osrmc_table_params_construct(osrmc_error_t* error);
OSRMC_API void
osrmc_table_params_destruct(osrmc_table_params_t params);
/* Table parameter setters*/
OSRMC_API void
osrmc_table_params_add_source(osrmc_table_params_t params, size_t index, osrmc_error_t* error);
OSRMC_API void
osrmc_table_params_add_destination(osrmc_table_params_t params, size_t index, osrmc_error_t* error);
OSRMC_API void
osrmc_table_params_set_annotations(osrmc_table_params_t params,
                                   osrmc_table_annotations_type_t annotations,
                                   osrmc_error_t* error);
OSRMC_API void
osrmc_table_params_set_fallback_speed(osrmc_table_params_t params, double speed, osrmc_error_t* error);
OSRMC_API void
osrmc_table_params_set_fallback_coordinate_type(osrmc_table_params_t params,
                                                osrmc_table_fallback_coordinate_type_t coord_type,
                                                osrmc_error_t* error);
OSRMC_API void
osrmc_table_params_set_scale_factor(osrmc_table_params_t params, double scale_factor, osrmc_error_t* error);
/* Table response constructor and destructor */
OSRMC_API osrmc_table_response_t
osrmc_table(osrmc_osrm_t osrm, osrmc_table_params_t params, osrmc_error_t* error);
OSRMC_API void
osrmc_table_response_destruct(osrmc_table_response_t response);
/* Table response getters*/
OSRMC_API osrmc_blob_t
osrmc_table_response_json(osrmc_table_response_t response, osrmc_error_t* error);

/* Match */

/* Match parameter constructor and destructor */
OSRMC_API osrmc_match_params_t
osrmc_match_params_construct(osrmc_error_t* error);
OSRMC_API void
osrmc_match_params_destruct(osrmc_match_params_t params);
/* Match parameter setters*/
OSRMC_API void
osrmc_match_params_set_steps(osrmc_match_params_t params, int on, osrmc_error_t* error);
OSRMC_API void
osrmc_match_params_set_alternatives(osrmc_match_params_t params, int on, osrmc_error_t* error);
OSRMC_API void
osrmc_match_params_set_geometries(osrmc_match_params_t params,
                                  osrmc_geometries_type_t geometries,
                                  osrmc_error_t* error);
OSRMC_API void
osrmc_match_params_set_overview(osrmc_match_params_t params, osrmc_overview_type_t overview, osrmc_error_t* error);
OSRMC_API void
osrmc_match_params_set_continue_straight(osrmc_match_params_t params, int on, osrmc_error_t* error);
OSRMC_API void
osrmc_match_params_set_number_of_alternatives(osrmc_match_params_t params, unsigned count, osrmc_error_t* error);
OSRMC_API void
osrmc_match_params_set_annotations(osrmc_match_params_t params,
                                   osrmc_annotations_type_t annotations,
                                   osrmc_error_t* error);
OSRMC_API void
osrmc_match_params_add_waypoint(osrmc_match_params_t params, size_t index, osrmc_error_t* error);
OSRMC_API void
osrmc_match_params_clear_waypoints(osrmc_match_params_t params);
OSRMC_API void
osrmc_match_params_add_timestamp(osrmc_match_params_t params, unsigned timestamp, osrmc_error_t* error);
OSRMC_API void
osrmc_match_params_set_gaps(osrmc_match_params_t params, osrmc_match_gaps_type_t gaps, osrmc_error_t* error);
OSRMC_API void
osrmc_match_params_set_tidy(osrmc_match_params_t params, int on, osrmc_error_t* error);
/* Match response constructor and destructor */
OSRMC_API osrmc_match_response_t
osrmc_match(osrmc_osrm_t osrm, osrmc_match_params_t params, osrmc_error_t* error);
OSRMC_API void
osrmc_match_response_destruct(osrmc_match_response_t response);
/* Match response getters*/
OSRMC_API osrmc_blob_t
osrmc_match_response_json(osrmc_match_response_t response, osrmc_error_t* error);

/* Trip */

/* Trip parameter constructor and destructor */
OSRMC_API osrmc_trip_params_t
osrmc_trip_params_construct(osrmc_error_t* error);
OSRMC_API void
osrmc_trip_params_destruct(osrmc_trip_params_t params);
/* Trip parameter setters*/
OSRMC_API void
osrmc_trip_params_set_roundtrip(osrmc_trip_params_t params, int on, osrmc_error_t* error);
OSRMC_API void
osrmc_trip_params_set_source(osrmc_trip_params_t params, osrmc_trip_source_type_t source, osrmc_error_t* error);
OSRMC_API void
osrmc_trip_params_set_destination(osrmc_trip_params_t params,
                                  osrmc_trip_destination_type_t destination,
                                  osrmc_error_t* error);
OSRMC_API void
osrmc_trip_params_set_steps(osrmc_trip_params_t params, int on, osrmc_error_t* error);
OSRMC_API void
osrmc_trip_params_set_alternatives(osrmc_trip_params_t params, int on, osrmc_error_t* error);
OSRMC_API void
osrmc_trip_params_set_geometries(osrmc_trip_params_t params, osrmc_geometries_type_t geometries, osrmc_error_t* error);
OSRMC_API void
osrmc_trip_params_set_overview(osrmc_trip_params_t params, osrmc_overview_type_t overview, osrmc_error_t* error);
OSRMC_API void
osrmc_trip_params_set_continue_straight(osrmc_trip_params_t params, int on, osrmc_error_t* error);
OSRMC_API void
osrmc_trip_params_set_number_of_alternatives(osrmc_trip_params_t params, unsigned count, osrmc_error_t* error);
OSRMC_API void
osrmc_trip_params_set_annotations(osrmc_trip_params_t params,
                                  osrmc_annotations_type_t annotations,
                                  osrmc_error_t* error);
OSRMC_API void
osrmc_trip_params_clear_waypoints(osrmc_trip_params_t params);
OSRMC_API void
osrmc_trip_params_add_waypoint(osrmc_trip_params_t params, size_t index, osrmc_error_t* error);
/* Trip response constructor and destructor */
OSRMC_API osrmc_trip_response_t
osrmc_trip(osrmc_osrm_t osrm, osrmc_trip_params_t params, osrmc_error_t* error);
OSRMC_API void
osrmc_trip_response_destruct(osrmc_trip_response_t response);
/* Trip response getters*/
OSRMC_API osrmc_blob_t
osrmc_trip_response_json(osrmc_trip_response_t response, osrmc_error_t* error);

/* Tile */

/* Tile parameter constructor and destructor */
OSRMC_API osrmc_tile_params_t
osrmc_tile_params_construct(osrmc_error_t* error);
OSRMC_API void
osrmc_tile_params_destruct(osrmc_tile_params_t params);
/* Tile parameter setters*/
OSRMC_API void
osrmc_tile_params_set_x(osrmc_tile_params_t params, unsigned x, osrmc_error_t* error);
OSRMC_API void
osrmc_tile_params_set_y(osrmc_tile_params_t params, unsigned y, osrmc_error_t* error);
OSRMC_API void
osrmc_tile_params_set_z(osrmc_tile_params_t params, unsigned z, osrmc_error_t* error);
/* Tile response constructor and destructor */
OSRMC_API osrmc_tile_response_t
osrmc_tile(osrmc_osrm_t osrm, osrmc_tile_params_t params, osrmc_error_t* error);
OSRMC_API void
osrmc_tile_response_destruct(osrmc_tile_response_t response);
/* Tile response getters*/
OSRMC_API const char*
osrmc_tile_response_data(osrmc_tile_response_t response, size_t* size, osrmc_error_t* error);
OSRMC_API size_t
osrmc_tile_response_size(osrmc_tile_response_t response, osrmc_error_t* error);

#ifdef __cplusplus
}
#endif

#endif
