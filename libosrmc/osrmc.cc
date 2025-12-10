// Standard library headers
#include <algorithm>
#include <cctype>
#include <cmath>
#include <exception>
#include <filesystem>
#include <limits>
#include <locale>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>
#include <variant>
#include <vector>

// OSRM backend headers
#include <osrm/engine/hint.hpp>
#include <osrm/bearing.hpp>
#include <osrm/coordinate.hpp>
#include <osrm/engine/api/base_parameters.hpp>
#include <osrm/engine/api/base_result.hpp>
#include <osrm/engine_config.hpp>
#include <osrm/json_container.hpp>
#include <osrm/match_parameters.hpp>
#include <osrm/nearest_parameters.hpp>
#include <osrm/osrm.hpp>
#include <osrm/route_parameters.hpp>
#include <osrm/status.hpp>
#include <osrm/storage_config.hpp>
#include <osrm/table_parameters.hpp>
#include <osrm/tile_parameters.hpp>
#include <osrm/trip_parameters.hpp>

// Local headers
#include "osrmc.h"

/* ABI stability */

unsigned
osrmc_get_version(void) {
  return OSRMC_VERSION;
}

int
osrmc_is_abi_compatible(void) {
  return osrmc_get_version() >> 16u == OSRMC_VERSION_MAJOR;
}

/* API */

struct osrmc_error final {
  std::string code;
  std::string message;
};

struct osrmc_blob final {
  std::string data;
};

struct osrmc_route_response final {
  osrm::json::Object json;
  osrm::RouteParameters::GeometriesType geometries = osrm::RouteParameters::GeometriesType::GeoJSON;
};

namespace {

  constexpr unsigned char JSON_CONTROL_CHAR_THRESHOLD = 0x20;
  constexpr unsigned char JSON_HEX_MASK = 0x0F;
  constexpr int JSON_NUMBER_PRECISION = 10;

  constexpr size_t COORDINATE_LONGITUDE_INDEX = 0;
  constexpr size_t COORDINATE_LATITUDE_INDEX = 1;
  constexpr size_t MIN_COORDINATE_PAIR_SIZE = 2;

  static bool osrmc_validate_config(osrmc_config_t config, osrmc_error_t* error) {
    if (!config) {
      if (error) {
        *error = new osrmc_error{"InvalidArgument", "Config must not be null"};
      }
      return false;
    }
    return true;
  }

  static bool osrmc_validate_params(osrmc_params_t params, osrmc_error_t* error) {
    if (!params) {
      if (error) {
        *error = new osrmc_error{"InvalidArgument", "Params must not be null"};
      }
      return false;
    }
    return true;
  }

  static bool osrmc_validate_osrm(osrmc_osrm_t osrm, osrmc_error_t* error) {
    if (!osrm) {
      if (error) {
        *error = new osrmc_error{"InvalidArgument", "OSRM instance must not be null"};
      }
      return false;
    }
    return true;
  }

  static bool osrmc_validate_nearest_response(osrmc_nearest_response_t response, osrmc_error_t* error) {
    if (!response) {
      if (error) {
        *error = new osrmc_error{"InvalidArgument", "Nearest response must not be null"};
      }
      return false;
    }
    return true;
  }

  static bool osrmc_validate_route_response(osrmc_route_response_t response, osrmc_error_t* error) {
    if (!response) {
      if (error) {
        *error = new osrmc_error{"InvalidArgument", "Route response must not be null"};
      }
      return false;
    }
    return true;
  }

  static bool osrmc_validate_table_response(osrmc_table_response_t response, osrmc_error_t* error) {
    if (!response) {
      if (error) {
        *error = new osrmc_error{"InvalidArgument", "Table response must not be null"};
      }
      return false;
    }
    return true;
  }

  static bool osrmc_validate_match_response(osrmc_match_response_t response, osrmc_error_t* error) {
    if (!response) {
      if (error) {
        *error = new osrmc_error{"InvalidArgument", "Match response must not be null"};
      }
      return false;
    }
    return true;
  }

  static bool osrmc_validate_trip_response(osrmc_trip_response_t response, osrmc_error_t* error) {
    if (!response) {
      if (error) {
        *error = new osrmc_error{"InvalidArgument", "Trip response must not be null"};
      }
      return false;
    }
    return true;
  }

  static bool osrmc_validate_tile_response(osrmc_tile_response_t response, osrmc_error_t* error) {
    if (!response) {
      if (error) {
        *error = new osrmc_error{"InvalidArgument", "Tile response must not be null"};
      }
      return false;
    }
    return true;
  }

  void osrmc_json_append_escaped(std::string& out, std::string_view value) {
    for (const unsigned char ch : value) {
      switch (ch) {
        case '"':
          out += "\\\"";
          break;
        case '\\':
          out += "\\\\";
          break;
        case '\b':
          out += "\\b";
          break;
        case '\f':
          out += "\\f";
          break;
        case '\n':
          out += "\\n";
          break;
        case '\r':
          out += "\\r";
          break;
        case '\t':
          out += "\\t";
          break;
        default:
          if (ch < JSON_CONTROL_CHAR_THRESHOLD) {
            constexpr char hex_digits[] = "0123456789abcdef";
            out += "\\u00";
            out.push_back(hex_digits[(ch >> 4) & JSON_HEX_MASK]);
            out.push_back(hex_digits[ch & JSON_HEX_MASK]);
          } else {
            out.push_back(static_cast<char>(ch));
          }
          break;
      }
    }
  }

  struct osrmc_json_renderer {
    std::string& out;

    void operator()(const osrm::json::String& value) const {
      out.push_back('"');
      osrmc_json_append_escaped(out, value.value);
      out.push_back('"');
    }

    void operator()(const osrm::json::Number& value) const {
      if (!std::isfinite(value.value)) {
        out += "null";
        return;
      }
      std::ostringstream stream;
      stream.imbue(std::locale::classic());
      stream.precision(JSON_NUMBER_PRECISION);
      stream << std::defaultfloat << value.value;
      out += stream.str();
    }

    void operator()(const osrm::json::Object& object) const {
      out.push_back('{');
      bool first = true;
      for (const auto& [key, child] : object.values) {
        if (!first) {
          out.push_back(',');
        }
        first = false;
        out.push_back('"');
        osrmc_json_append_escaped(out, key);
        out.push_back('"');
        out.push_back(':');
        std::visit(osrmc_json_renderer{out}, child);
      }
      out.push_back('}');
    }

    void operator()(const osrm::json::Array& array) const {
      out.push_back('[');
      bool first = true;
      for (const auto& child : array.values) {
        if (!first) {
          out.push_back(',');
        }
        first = false;
        std::visit(osrmc_json_renderer{out}, child);
      }
      out.push_back(']');
    }

    void operator()(const osrm::json::True&) const { out += "true"; }
    void operator()(const osrm::json::False&) const { out += "false"; }
    void operator()(const osrm::json::Null&) const { out += "null"; }
  };

  void osrmc_render_json(std::string& out, const osrm::json::Object& object) {
    out.clear();
    osrmc_json_renderer renderer{out};
    renderer(object);
  }

} // namespace

template<typename T>
static bool
osrmc_validate_coordinate_index(const T& params, size_t coordinate_index, const char* parameter, osrmc_error_t* error) {
  if (coordinate_index >= params.coordinates.size()) {
    if (error) {
      *error = new osrmc_error{"InvalidCoordinateIndex", std::string(parameter) + " index out of bounds"};
    }
    return false;
  }
  return true;
}

template<typename Container>
static void
osrmc_ensure_container_size(Container& container, size_t size) {
  if (container.size() < size) {
    container.resize(size);
  }
}

static std::string
osrmc_to_lower(std::string value) {
  std::transform(
    value.begin(), value.end(), value.begin(), [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
  return value;
}

static std::optional<osrm::storage::FeatureDataset>
osrmc_feature_dataset_from_string(const std::string& name) {
  const auto lower = osrmc_to_lower(name);
  if (lower == "route_steps") {
    return osrm::storage::FeatureDataset::ROUTE_STEPS;
  }
  if (lower == "route_geometry") {
    return osrm::storage::FeatureDataset::ROUTE_GEOMETRY;
  }
  return std::nullopt;
}

static osrmc_blob_t
osrmc_render_json(const osrm::json::Object& object) {
  auto* blob = new osrmc_blob;
  osrmc_render_json(blob->data, object);
  return reinterpret_cast<osrmc_blob_t>(blob);
}

static void
osrmc_error_from_exception(const std::exception& e, osrmc_error_t* error) {
  if (error) {
    *error = new osrmc_error{"Exception", e.what()};
  }
}

static void
osrmc_error_from_json(osrm::json::Object& json, osrmc_error_t* error) try {
  if (!error) {
    return;
  }
  auto code = std::get<osrm::json::String>(json.values["code"]).value;
  auto message = std::get<osrm::json::String>(json.values["message"]).value;
  if (code.empty()) {
    code = "Unknown";
  }

  *error = new osrmc_error{code, message};
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

/* Error handling */

const char*
osrmc_error_code(osrmc_error_t error) {
  return error->code.c_str();
}

const char*
osrmc_error_message(osrmc_error_t error) {
  return error->message.c_str();
}

void
osrmc_error_destruct(osrmc_error_t error) {
  if (error) {
    delete error;
  }
}

/* Config */

osrmc_config_t
osrmc_config_construct(const char* base_path, osrmc_error_t* error) try {
  auto* out = new osrm::EngineConfig;

  if (base_path) {
    out->storage_config = osrm::StorageConfig(std::filesystem::path(base_path));
    out->use_shared_memory = false;
  } else {
    out->use_shared_memory = true;
  }

  return reinterpret_cast<osrmc_config_t>(out);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

void
osrmc_config_destruct(osrmc_config_t config) {
  if (config) {
    delete reinterpret_cast<osrm::EngineConfig*>(config);
  }
}

void
osrmc_config_set_max_locations_trip(osrmc_config_t config, int max_locations, osrmc_error_t* error) try {
  if (!osrmc_validate_config(config, error)) {
    return;
  }
  auto* config_typed = reinterpret_cast<osrm::EngineConfig*>(config);
  config_typed->max_locations_trip = max_locations;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_config_set_max_locations_viaroute(osrmc_config_t config, int max_locations, osrmc_error_t* error) try {
  if (!osrmc_validate_config(config, error)) {
    return;
  }
  auto* config_typed = reinterpret_cast<osrm::EngineConfig*>(config);
  config_typed->max_locations_viaroute = max_locations;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_config_set_max_locations_distance_table(osrmc_config_t config, int max_locations, osrmc_error_t* error) try {
  if (!osrmc_validate_config(config, error)) {
    return;
  }
  auto* config_typed = reinterpret_cast<osrm::EngineConfig*>(config);
  config_typed->max_locations_distance_table = max_locations;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_config_set_max_locations_map_matching(osrmc_config_t config, int max_locations, osrmc_error_t* error) try {
  if (!osrmc_validate_config(config, error)) {
    return;
  }
  auto* config_typed = reinterpret_cast<osrm::EngineConfig*>(config);
  config_typed->max_locations_map_matching = max_locations;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_config_set_max_radius_map_matching(osrmc_config_t config, double max_radius, osrmc_error_t* error) try {
  if (!osrmc_validate_config(config, error)) {
    return;
  }
  auto* config_typed = reinterpret_cast<osrm::EngineConfig*>(config);
  config_typed->max_radius_map_matching = max_radius;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_config_set_max_results_nearest(osrmc_config_t config, int max_results, osrmc_error_t* error) try {
  if (!osrmc_validate_config(config, error)) {
    return;
  }
  auto* config_typed = reinterpret_cast<osrm::EngineConfig*>(config);
  config_typed->max_results_nearest = max_results;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_config_set_default_radius(osrmc_config_t config, double default_radius, osrmc_error_t* error) try {
  if (!osrmc_validate_config(config, error)) {
    return;
  }
  auto* config_typed = reinterpret_cast<osrm::EngineConfig*>(config);
  config_typed->default_radius = default_radius;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_config_set_max_alternatives(osrmc_config_t config, int max_alternatives, osrmc_error_t* error) try {
  if (!osrmc_validate_config(config, error)) {
    return;
  }
  auto* config_typed = reinterpret_cast<osrm::EngineConfig*>(config);
  config_typed->max_alternatives = max_alternatives;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_config_set_use_shared_memory(osrmc_config_t config, bool use_shared_memory, osrmc_error_t* error) try {
  if (!osrmc_validate_config(config, error)) {
    return;
  }
  auto* config_typed = reinterpret_cast<osrm::EngineConfig*>(config);
  config_typed->use_shared_memory = use_shared_memory;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_config_set_memory_file(osrmc_config_t config, const char* memory_file, osrmc_error_t* error) try {
  if (!osrmc_validate_config(config, error)) {
    return;
  }
  auto* config_typed = reinterpret_cast<osrm::EngineConfig*>(config);
  if (memory_file) {
    config_typed->memory_file = std::filesystem::path(memory_file);
  } else {
    config_typed->memory_file.clear();
  }
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_config_set_use_mmap(osrmc_config_t config, bool use_mmap, osrmc_error_t* error) try {
  if (!osrmc_validate_config(config, error)) {
    return;
  }
  auto* config_typed = reinterpret_cast<osrm::EngineConfig*>(config);
  config_typed->use_mmap = use_mmap;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_config_set_algorithm(osrmc_config_t config, osrmc_algorithm_t algorithm, osrmc_error_t* error) try {
  if (!osrmc_validate_config(config, error)) {
    return;
  }
  auto* config_typed = reinterpret_cast<osrm::EngineConfig*>(config);

  switch (algorithm) {
    case OSRMC_ALGORITHM_CH:
      config_typed->algorithm = osrm::EngineConfig::Algorithm::CH;
      break;
    case OSRMC_ALGORITHM_MLD:
      config_typed->algorithm = osrm::EngineConfig::Algorithm::MLD;
      break;
    default:
      *error = new osrmc_error{"InvalidAlgorithm", "Unknown algorithm type"};
      return;
  }
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

static void
osrmc_refresh_storage_config_for_datasets(osrm::EngineConfig* config_typed) {
  const auto base_path = config_typed->storage_config.base_path;
  if (!base_path.empty()) {
    config_typed->storage_config = osrm::StorageConfig(base_path, config_typed->disable_feature_dataset);
  } else {
    config_typed->storage_config = osrm::StorageConfig(config_typed->disable_feature_dataset);
  }
}

void
osrmc_config_disable_feature_dataset(osrmc_config_t config, const char* dataset_name, osrmc_error_t* error) try {
  if (!osrmc_validate_config(config, error)) {
    return;
  }
  if (!dataset_name) {
    if (error) {
      *error = new osrmc_error{"InvalidDataset", "Dataset name must not be null"};
    }
    return;
  }

  auto* config_typed = reinterpret_cast<osrm::EngineConfig*>(config);
  const auto dataset = osrmc_feature_dataset_from_string(dataset_name);
  if (!dataset) {
    *error = new osrmc_error{"InvalidDataset", "Unknown dataset"};
    return;
  }

  const auto exists =
    std::find(config_typed->disable_feature_dataset.begin(), config_typed->disable_feature_dataset.end(), *dataset) !=
    config_typed->disable_feature_dataset.end();
  if (!exists) {
    config_typed->disable_feature_dataset.push_back(*dataset);
    osrmc_refresh_storage_config_for_datasets(config_typed);
  }
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_config_set_verbosity(osrmc_config_t config, const char* verbosity, osrmc_error_t* error) try {
  if (!osrmc_validate_config(config, error)) {
    return;
  }
  auto* config_typed = reinterpret_cast<osrm::EngineConfig*>(config);
  config_typed->verbosity = verbosity ? verbosity : "";
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_config_set_dataset_name(osrmc_config_t config, const char* dataset_name, osrmc_error_t* error) try {
  if (!osrmc_validate_config(config, error)) {
    return;
  }
  auto* config_typed = reinterpret_cast<osrm::EngineConfig*>(config);
  config_typed->dataset_name = dataset_name ? dataset_name : "";
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_config_clear_disabled_feature_datasets(osrmc_config_t config, osrmc_error_t* error) try {
  if (!osrmc_validate_config(config, error)) {
    return;
  }
  auto* config_typed = reinterpret_cast<osrm::EngineConfig*>(config);
  config_typed->disable_feature_dataset.clear();
  osrmc_refresh_storage_config_for_datasets(config_typed);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

/* OSRM */

osrmc_osrm_t
osrmc_osrm_construct(osrmc_config_t config, osrmc_error_t* error) try {
  if (!config) {
    if (error) {
      *error = new osrmc_error{"InvalidArgument", "Config must not be null"};
    }
    return nullptr;
  }
  auto* config_typed = reinterpret_cast<osrm::EngineConfig*>(config);
  auto* out = new osrm::OSRM(*config_typed);

  return reinterpret_cast<osrmc_osrm_t>(out);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

void
osrmc_osrm_destruct(osrmc_osrm_t osrm) {
  if (osrm) {
    delete reinterpret_cast<osrm::OSRM*>(osrm);
  }
}

void
osrmc_params_add_coordinate(osrmc_params_t params, double longitude, double latitude, osrmc_error_t* error) try {
  if (!osrmc_validate_params(params, error)) {
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::engine::api::BaseParameters*>(params);

  params_typed->coordinates.emplace_back(osrm::util::FloatLongitude{longitude}, osrm::util::FloatLatitude{latitude});
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_params_add_coordinate_with(osrmc_params_t params,
                                 double longitude,
                                 double latitude,
                                 double radius,
                                 int bearing,
                                 int range,
                                 osrmc_error_t* error) try {
  if (!osrmc_validate_params(params, error)) {
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::engine::api::BaseParameters*>(params);

  params_typed->coordinates.emplace_back(osrm::util::FloatLongitude{longitude}, osrm::util::FloatLatitude{latitude});
  params_typed->radiuses.emplace_back(radius);
  params_typed->bearings.emplace_back(osrm::Bearing{static_cast<short>(bearing), static_cast<short>(range)});
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_params_set_hint(osrmc_params_t params,
                      size_t coordinate_index,
                      const char* hint_base64,
                      osrmc_error_t* error) try {
  if (!osrmc_validate_params(params, error)) {
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::engine::api::BaseParameters*>(params);
  if (!osrmc_validate_coordinate_index(*params_typed, coordinate_index, "Hint", error)) {
    return;
  }

  osrmc_ensure_container_size(params_typed->hints, params_typed->coordinates.size());
  if (hint_base64) {
    auto hint = osrm::engine::Hint::FromBase64(hint_base64);
    params_typed->hints[coordinate_index] = std::move(hint);
  } else {
    params_typed->hints[coordinate_index] = std::nullopt;
  }
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_params_set_radius(osrmc_params_t params, size_t coordinate_index, double radius, osrmc_error_t* error) try {
  if (!osrmc_validate_params(params, error)) {
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::engine::api::BaseParameters*>(params);
  if (!osrmc_validate_coordinate_index(*params_typed, coordinate_index, "Radius", error)) {
    return;
  }

  osrmc_ensure_container_size(params_typed->radiuses, params_typed->coordinates.size());
  if (radius >= 0.0) {
    params_typed->radiuses[coordinate_index] = radius;
  } else {
    params_typed->radiuses[coordinate_index] = std::nullopt;
  }
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_params_set_bearing(osrmc_params_t params,
                         size_t coordinate_index,
                         int value,
                         int range,
                         osrmc_error_t* error) try {
  if (!osrmc_validate_params(params, error)) {
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::engine::api::BaseParameters*>(params);
  if (!osrmc_validate_coordinate_index(*params_typed, coordinate_index, "Bearing", error)) {
    return;
  }

  osrmc_ensure_container_size(params_typed->bearings, params_typed->coordinates.size());
  if (value < 0 || range < 0) {
    params_typed->bearings[coordinate_index] = std::nullopt;
    return;
  }

  params_typed->bearings[coordinate_index] = osrm::Bearing{static_cast<short>(value), static_cast<short>(range)};
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_params_set_approach(osrmc_params_t params,
                          size_t coordinate_index,
                          osrmc_approach_t approach,
                          osrmc_error_t* error) try {
  if (!osrmc_validate_params(params, error)) {
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::engine::api::BaseParameters*>(params);
  if (!osrmc_validate_coordinate_index(*params_typed, coordinate_index, "Approach", error)) {
    return;
  }

  osrmc_ensure_container_size(params_typed->approaches, params_typed->coordinates.size());
  std::optional<osrm::engine::Approach> approach_value;
  switch (approach) {
    case OSRMC_APPROACH_CURB:
      approach_value = osrm::engine::Approach::CURB;
      break;
    case OSRMC_APPROACH_UNRESTRICTED:
      approach_value = osrm::engine::Approach::UNRESTRICTED;
      break;
    case OSRMC_APPROACH_OPPOSITE:
      approach_value = osrm::engine::Approach::OPPOSITE;
      break;
    default:
      approach_value = std::nullopt;
      break;
  }

  params_typed->approaches[coordinate_index] = std::move(approach_value);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_params_add_exclude(osrmc_params_t params, const char* exclude_profile, osrmc_error_t* error) try {
  if (!osrmc_validate_params(params, error)) {
    return;
  }
  if (!exclude_profile) {
    if (error) {
      *error = new osrmc_error{"InvalidExclude", "Exclude profile must not be null"};
    }
    return;
  }

  auto* params_typed = reinterpret_cast<osrm::engine::api::BaseParameters*>(params);
  params_typed->exclude.emplace_back(exclude_profile);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_params_set_generate_hints(osrmc_params_t params, int on, osrmc_error_t* error) try {
  if (!osrmc_validate_params(params, error)) {
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::engine::api::BaseParameters*>(params);
  params_typed->generate_hints = on != 0;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_params_set_skip_waypoints(osrmc_params_t params, int on, osrmc_error_t* error) try {
  if (!osrmc_validate_params(params, error)) {
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::engine::api::BaseParameters*>(params);
  params_typed->skip_waypoints = on != 0;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_params_set_snapping(osrmc_params_t params, osrmc_snapping_t snapping, osrmc_error_t* error) try {
  if (!osrmc_validate_params(params, error)) {
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::engine::api::BaseParameters*>(params);
  switch (snapping) {
    case OSRMC_SNAPPING_DEFAULT:
      params_typed->snapping = osrm::engine::api::BaseParameters::SnappingType::Default;
      break;
    case OSRMC_SNAPPING_ANY:
      params_typed->snapping = osrm::engine::api::BaseParameters::SnappingType::Any;
      break;
    default:
      *error = new osrmc_error{"InvalidSnapping", "Unknown snapping type"};
      return;
  }
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_params_set_format(osrmc_params_t params, osrmc_output_format_t format, osrmc_error_t* error) try {
  if (!osrmc_validate_params(params, error)) {
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::engine::api::BaseParameters*>(params);
  switch (format) {
    case OSRMC_FORMAT_JSON:
      params_typed->format = osrm::engine::api::BaseParameters::OutputFormatType::JSON;
      break;
    default:
      if (error) {
        *error = new osrmc_error{"InvalidFormat", "Unknown output format"};
      }
      return;
  }
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

static osrm::RouteParameters*
osrmc_route_like_params(osrmc_route_params_t params) {
  return reinterpret_cast<osrm::RouteParameters*>(params);
}

static osrm::RouteParameters*
osrmc_route_like_params(osrmc_match_params_t params) {
  return static_cast<osrm::RouteParameters*>(reinterpret_cast<osrm::MatchParameters*>(params));
}

static osrm::RouteParameters*
osrmc_route_like_params(osrmc_trip_params_t params) {
  return static_cast<osrm::RouteParameters*>(reinterpret_cast<osrm::TripParameters*>(params));
}

template<typename ParamsHandle>
static void
osrmc_route_like_set_steps_flag(ParamsHandle params, int on) {
  if (!params) {
    return;
  }
  osrmc_route_like_params(params)->steps = on != 0;
}

template<typename ParamsHandle>
static void
osrmc_route_like_set_alternatives_flag(ParamsHandle params, int on) {
  if (!params) {
    return;
  }
  osrmc_route_like_params(params)->alternatives = on != 0;
}

template<typename ParamsHandle>
static void
osrmc_route_like_set_number_of_alternatives_value(ParamsHandle params, unsigned count) {
  if (!params) {
    return;
  }
  auto* params_typed = osrmc_route_like_params(params);
  params_typed->number_of_alternatives = count;
  params_typed->alternatives = count > 0;
}

static void
osrmc_route_like_set_continue_straight_impl(osrm::RouteParameters* params, int on) {
  if (!params) {
    return;
  }
  if (on < 0) {
    params->continue_straight = std::nullopt;
  } else {
    params->continue_straight = (on != 0);
  }
}


static void
osrmc_route_like_add_waypoint_impl(osrm::RouteParameters* params, size_t index) {
  if (!params) {
    return;
  }
  params->waypoints.emplace_back(index);
}

static void
osrmc_route_like_clear_waypoints_impl(osrm::RouteParameters* params) {
  if (!params) {
    return;
  }
  params->waypoints.clear();
}

static osrmc_route_response*
osrmc_get_route_response(osrmc_route_response_t response) {
  return reinterpret_cast<osrmc_route_response*>(response);
}

static const osrm::json::Object*
osrmc_get_route_step(const osrm::json::Object& response_json,
                     unsigned route_index,
                     unsigned leg_index,
                     unsigned step_index,
                     osrmc_error_t* error) {
  const auto& routes = std::get<osrm::json::Array>(response_json.values.at("routes"));
  if (route_index >= routes.values.size()) {
    if (error) {
      *error = new osrmc_error{"IndexOutOfBounds", "Route index out of bounds"};
    }
    return nullptr;
  }

  const auto& route = std::get<osrm::json::Object>(routes.values.at(route_index));
  const auto& legs = std::get<osrm::json::Array>(route.values.at("legs"));
  if (leg_index >= legs.values.size()) {
    if (error) {
      *error = new osrmc_error{"IndexOutOfBounds", "Leg index out of bounds"};
    }
    return nullptr;
  }

  const auto& leg = std::get<osrm::json::Object>(legs.values.at(leg_index));
  const auto steps_iter = leg.values.find("steps");
  if (steps_iter == leg.values.end()) {
    if (error) {
      *error = new osrmc_error{"NoSteps", "Steps not available for this leg"};
    }
    return nullptr;
  }

  const auto& steps = std::get<osrm::json::Array>(steps_iter->second);
  if (step_index >= steps.values.size()) {
    if (error) {
      *error = new osrmc_error{"IndexOutOfBounds", "Step index out of bounds"};
    }
    return nullptr;
  }

  return &std::get<osrm::json::Object>(steps.values.at(step_index));
}

static const osrm::json::Object*
osrmc_get_match_step(const osrm::json::Object& response_json,
                     unsigned route_index,
                     unsigned leg_index,
                     unsigned step_index,
                     osrmc_error_t* error) {
  const auto& matchings = std::get<osrm::json::Array>(response_json.values.at("matchings"));
  if (route_index >= matchings.values.size()) {
    if (error) {
      *error = new osrmc_error{"IndexOutOfBounds", "Route index out of bounds"};
    }
    return nullptr;
  }

  const auto& route = std::get<osrm::json::Object>(matchings.values.at(route_index));
  const auto& legs = std::get<osrm::json::Array>(route.values.at("legs"));
  if (leg_index >= legs.values.size()) {
    if (error) {
      *error = new osrmc_error{"IndexOutOfBounds", "Leg index out of bounds"};
    }
    return nullptr;
  }

  const auto& leg = std::get<osrm::json::Object>(legs.values.at(leg_index));
  const auto steps_iter = leg.values.find("steps");
  if (steps_iter == leg.values.end()) {
    if (error) {
      *error = new osrmc_error{"NoSteps", "Steps not available for this leg"};
    }
    return nullptr;
  }

  const auto& steps = std::get<osrm::json::Array>(steps_iter->second);
  if (step_index >= steps.values.size()) {
    if (error) {
      *error = new osrmc_error{"IndexOutOfBounds", "Step index out of bounds"};
    }
    return nullptr;
  }

  return &std::get<osrm::json::Object>(steps.values.at(step_index));
}

static const osrm::json::Object*
osrmc_get_trip_step(const osrm::json::Object& response_json,
                    unsigned trip_index,
                    unsigned leg_index,
                    unsigned step_index,
                    osrmc_error_t* error) {
  const auto& trips = std::get<osrm::json::Array>(response_json.values.at("trips"));
  if (trip_index >= trips.values.size()) {
    if (error) {
      *error = new osrmc_error{"IndexOutOfBounds", "Trip index out of bounds"};
    }
    return nullptr;
  }

  const auto& trip = std::get<osrm::json::Object>(trips.values.at(trip_index));
  const auto& legs = std::get<osrm::json::Array>(trip.values.at("legs"));
  if (leg_index >= legs.values.size()) {
    if (error) {
      *error = new osrmc_error{"IndexOutOfBounds", "Leg index out of bounds"};
    }
    return nullptr;
  }

  const auto& leg = std::get<osrm::json::Object>(legs.values.at(leg_index));
  const auto steps_iter = leg.values.find("steps");
  if (steps_iter == leg.values.end()) {
    if (error) {
      *error = new osrmc_error{"NoSteps", "Steps not available for this leg"};
    }
    return nullptr;
  }

  const auto& steps = std::get<osrm::json::Array>(steps_iter->second);
  if (step_index >= steps.values.size()) {
    if (error) {
      *error = new osrmc_error{"IndexOutOfBounds", "Step index out of bounds"};
    }
    return nullptr;
  }

  return &std::get<osrm::json::Object>(steps.values.at(step_index));
}

static const osrm::json::Object*
osrmc_get_route_leg(const osrm::json::Object& response_json,
                    unsigned route_index,
                    unsigned leg_index,
                    osrmc_error_t* error) {
  const auto& routes = std::get<osrm::json::Array>(response_json.values.at("routes"));
  if (route_index >= routes.values.size()) {
    if (error) {
      *error = new osrmc_error{"IndexOutOfBounds", "Route index out of bounds"};
    }
    return nullptr;
  }

  const auto& route = std::get<osrm::json::Object>(routes.values.at(route_index));
  const auto& legs = std::get<osrm::json::Array>(route.values.at("legs"));
  if (leg_index >= legs.values.size()) {
    if (error) {
      *error = new osrmc_error{"IndexOutOfBounds", "Leg index out of bounds"};
    }
    return nullptr;
  }

  return &std::get<osrm::json::Object>(legs.values.at(leg_index));
}

static const osrm::json::Object*
osrmc_get_match_leg(const osrm::json::Object& response_json,
                    unsigned route_index,
                    unsigned leg_index,
                    osrmc_error_t* error) {
  const auto& matchings = std::get<osrm::json::Array>(response_json.values.at("matchings"));
  if (route_index >= matchings.values.size()) {
    if (error) {
      *error = new osrmc_error{"IndexOutOfBounds", "Route index out of bounds"};
    }
    return nullptr;
  }

  const auto& route = std::get<osrm::json::Object>(matchings.values.at(route_index));
  const auto& legs = std::get<osrm::json::Array>(route.values.at("legs"));
  if (leg_index >= legs.values.size()) {
    if (error) {
      *error = new osrmc_error{"IndexOutOfBounds", "Leg index out of bounds"};
    }
    return nullptr;
  }

  return &std::get<osrm::json::Object>(legs.values.at(leg_index));
}

static const osrm::json::Object*
osrmc_get_trip_leg(const osrm::json::Object& response_json,
                   unsigned trip_index,
                   unsigned leg_index,
                   osrmc_error_t* error) {
  const auto& trips = std::get<osrm::json::Array>(response_json.values.at("trips"));
  if (trip_index >= trips.values.size()) {
    if (error) {
      *error = new osrmc_error{"IndexOutOfBounds", "Trip index out of bounds"};
    }
    return nullptr;
  }

  const auto& trip = std::get<osrm::json::Object>(trips.values.at(trip_index));
  const auto& legs = std::get<osrm::json::Array>(trip.values.at("legs"));
  if (leg_index >= legs.values.size()) {
    if (error) {
      *error = new osrmc_error{"IndexOutOfBounds", "Leg index out of bounds"};
    }
    return nullptr;
  }

  return &std::get<osrm::json::Object>(legs.values.at(leg_index));
}

static const osrm::json::Object*
osrmc_get_step_intersection(const osrm::json::Object& step, unsigned intersection_index, osrmc_error_t* error) {
  const auto intersections_iter = step.values.find("intersections");
  if (intersections_iter == step.values.end()) {
    if (error) {
      *error = new osrmc_error{"NoIntersections", "Intersections not available for this step"};
    }
    return nullptr;
  }

  const auto& intersections = std::get<osrm::json::Array>(intersections_iter->second);
  if (intersection_index >= intersections.values.size()) {
    if (error) {
      *error = new osrmc_error{"IndexOutOfBounds", "Intersection index out of bounds"};
    }
    return nullptr;
  }

  return &std::get<osrm::json::Object>(intersections.values.at(intersection_index));
}

static const osrm::json::Array*
osrmc_get_step_geometry_coordinates(const osrm::json::Object& step, osrmc_error_t* error) {
  const auto geometry_iter = step.values.find("geometry");
  if (geometry_iter == step.values.end()) {
    if (error) {
      *error = new osrmc_error{"NoGeometry", "Geometry not available for this step"};
    }
    return nullptr;
  }

  const auto& geometry = geometry_iter->second;

  // Check if it's a polyline string
  if (std::holds_alternative<osrm::json::String>(geometry)) {
    if (error) {
      *error =
        new osrmc_error{"UnsupportedGeometry", "Step geometry is polyline string, use step_geometry_polyline instead"};
    }
    return nullptr;
  }

  // Check if it's GeoJSON coordinates array
  if (std::holds_alternative<osrm::json::Object>(geometry)) {
    const auto& geometry_obj = std::get<osrm::json::Object>(geometry);
    const auto coordinates_iter = geometry_obj.values.find("coordinates");
    if (coordinates_iter != geometry_obj.values.end()) {
      return &std::get<osrm::json::Array>(coordinates_iter->second);
    }
  }

  if (error) {
    *error = new osrmc_error{"UnsupportedGeometry", "Only GeoJSON geometry format is supported for coordinate access"};
  }
  return nullptr;
}

static const osrm::json::Array*
osrmc_get_route_geometry_coordinates(const osrm::json::Object& route, osrmc_error_t* error) {
  const auto geometry_iter = route.values.find("geometry");
  if (geometry_iter == route.values.end()) {
    return nullptr;
  }

  const auto& geometry = geometry_iter->second;

  if (std::holds_alternative<osrm::json::Object>(geometry)) {
    const auto& geometry_obj = std::get<osrm::json::Object>(geometry);
    const auto coordinates_iter = geometry_obj.values.find("coordinates");
    if (coordinates_iter != geometry_obj.values.end()) {
      return &std::get<osrm::json::Array>(coordinates_iter->second);
    }
  }

  if (error) {
    *error = new osrmc_error{"UnsupportedGeometry", "Only GeoJSON geometry format is supported"};
  }
  return nullptr;
}


const char*
osrmc_blob_data(osrmc_blob_t blob) {
  if (!blob) {
    return nullptr;
  }
  return reinterpret_cast<osrmc_blob*>(blob)->data.c_str();
}

size_t
osrmc_blob_size(osrmc_blob_t blob) {
  if (!blob) {
    return 0;
  }
  return reinterpret_cast<osrmc_blob*>(blob)->data.size();
}

void
osrmc_blob_destruct(osrmc_blob_t blob) {
  if (blob) {
    delete reinterpret_cast<osrmc_blob*>(blob);
  }
}

/* Nearest */

osrmc_nearest_params_t
osrmc_nearest_params_construct(osrmc_error_t* error) try {
  auto* out = new osrm::NearestParameters;
  return reinterpret_cast<osrmc_nearest_params_t>(out);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

void
osrmc_nearest_params_destruct(osrmc_nearest_params_t params) {
  if (params) {
    delete reinterpret_cast<osrm::NearestParameters*>(params);
  }
}

void
osrmc_nearest_params_set_number_of_results(osrmc_nearest_params_t params, unsigned n, osrmc_error_t* error) try {
  if (!osrmc_validate_params(reinterpret_cast<osrmc_params_t>(params), error)) {
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::NearestParameters*>(params);
  params_typed->number_of_results = n;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

osrmc_nearest_response_t
osrmc_nearest(osrmc_osrm_t osrm, osrmc_nearest_params_t params, osrmc_error_t* error) try {
  if (!osrmc_validate_osrm(osrm, error) || !params) {
    if (error) {
      *error = new osrmc_error{"InvalidArgument", "OSRM instance and params must not be null"};
    }
    return nullptr;
  }
  auto* osrm_typed = reinterpret_cast<osrm::OSRM*>(osrm);
  auto* params_typed = reinterpret_cast<osrm::NearestParameters*>(params);

  osrm::engine::api::ResultT result = osrm::json::Object();
  const auto status = osrm_typed->Nearest(*params_typed, result);

  if (status == osrm::Status::Ok) {
    if (std::holds_alternative<osrm::json::Object>(result)) {
      auto* out = new osrm::json::Object(std::move(std::get<osrm::json::Object>(result)));
      return reinterpret_cast<osrmc_nearest_response_t>(out);
    } else {
      if (error) {
        *error = new osrmc_error{"InvalidResponse", "Unexpected response type"};
      }
      return nullptr;
    }
  }

  if (std::holds_alternative<osrm::json::Object>(result)) {
    osrmc_error_from_json(std::get<osrm::json::Object>(result), error);
  } else {
    if (error) {
      *error = new osrmc_error{"NearestError", "Nearest request failed"};
    }
  }
  return nullptr;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

void
osrmc_nearest_response_destruct(osrmc_nearest_response_t response) {
  if (response) {
    delete reinterpret_cast<osrm::json::Object*>(response);
  }
}

unsigned
osrmc_nearest_response_count(osrmc_nearest_response_t response, osrmc_error_t* error) try {
  if (!osrmc_validate_nearest_response(response, error)) {
    return 0;
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  if (response_typed->values.find("waypoints") == response_typed->values.end()) {
    return 0;
  }

  const auto& waypoints = std::get<osrm::json::Array>(response_typed->values.at("waypoints"));
  return static_cast<unsigned>(waypoints.values.size());
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return 0;
}

double
osrmc_nearest_response_latitude(osrmc_nearest_response_t response, unsigned index, osrmc_error_t* error) try {
  if (!osrmc_validate_nearest_response(response, error)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  const auto& waypoints = std::get<osrm::json::Array>(response_typed->values.at("waypoints"));
  if (index >= waypoints.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Waypoint index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& waypoint = std::get<osrm::json::Object>(waypoints.values.at(index));
  const auto& location = std::get<osrm::json::Array>(waypoint.values.at("location")).values;
  const auto latitude = std::get<osrm::json::Number>(location[COORDINATE_LATITUDE_INDEX]).value;

  return latitude;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::quiet_NaN();
}

double
osrmc_nearest_response_longitude(osrmc_nearest_response_t response, unsigned index, osrmc_error_t* error) try {
  if (!osrmc_validate_nearest_response(response, error)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  const auto& waypoints = std::get<osrm::json::Array>(response_typed->values.at("waypoints"));
  if (index >= waypoints.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Waypoint index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& waypoint = std::get<osrm::json::Object>(waypoints.values.at(index));
  const auto& location = std::get<osrm::json::Array>(waypoint.values.at("location")).values;
  const auto longitude = std::get<osrm::json::Number>(location[COORDINATE_LONGITUDE_INDEX]).value;

  return longitude;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::quiet_NaN();
}

const char*
osrmc_nearest_response_name(osrmc_nearest_response_t response, unsigned index, osrmc_error_t* error) try {
  if (!osrmc_validate_nearest_response(response, error)) {
    return nullptr;
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  const auto& waypoints = std::get<osrm::json::Array>(response_typed->values.at("waypoints"));
  if (index >= waypoints.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Waypoint index out of bounds"};
    return nullptr;
  }

  const auto& waypoint = std::get<osrm::json::Object>(waypoints.values.at(index));
  const auto& name = std::get<osrm::json::String>(waypoint.values.at("name")).value;

  // Valid as long as response object exists
  return name.c_str();
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

double
osrmc_nearest_response_distance(osrmc_nearest_response_t response, unsigned index, osrmc_error_t* error) try {
  if (!osrmc_validate_nearest_response(response, error)) {
    return std::numeric_limits<double>::infinity();
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  const auto& waypoints = std::get<osrm::json::Array>(response_typed->values.at("waypoints"));
  if (index >= waypoints.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Waypoint index out of bounds"};
    return std::numeric_limits<double>::infinity();
  }

  const auto& waypoint = std::get<osrm::json::Object>(waypoints.values.at(index));
  const auto distance = std::get<osrm::json::Number>(waypoint.values.at("distance")).value;

  return distance;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::infinity();
}

const char*
osrmc_nearest_response_hint(osrmc_nearest_response_t response, unsigned index, osrmc_error_t* error) try {
  if (!osrmc_validate_nearest_response(response, error)) {
    return nullptr;
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  const auto& waypoints = std::get<osrm::json::Array>(response_typed->values.at("waypoints"));
  if (index >= waypoints.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Waypoint index out of bounds"};
    return nullptr;
  }

  const auto& waypoint = std::get<osrm::json::Object>(waypoints.values.at(index));
  const auto hint_iter = waypoint.values.find("hint");
  if (hint_iter == waypoint.values.end()) {
    *error = new osrmc_error{"NoHint", "Hint not available for this waypoint"};
    return nullptr;
  }

  const auto& hint = std::get<osrm::json::String>(hint_iter->second).value;
  return hint.c_str();
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

unsigned long long
osrmc_nearest_response_from_node(osrmc_nearest_response_t response, unsigned index, osrmc_error_t* error) try {
  if (!osrmc_validate_nearest_response(response, error)) {
    return 0;
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  const auto& waypoints = std::get<osrm::json::Array>(response_typed->values.at("waypoints"));
  if (index >= waypoints.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Waypoint index out of bounds"};
    return 0;
  }

  const auto& waypoint = std::get<osrm::json::Object>(waypoints.values.at(index));
  const auto nodes_iter = waypoint.values.find("nodes");
  if (nodes_iter == waypoint.values.end()) {
    *error = new osrmc_error{"NoNodes", "Nodes not available for this waypoint"};
    return 0;
  }

  const auto& nodes = std::get<osrm::json::Array>(nodes_iter->second);
  if (nodes.values.size() < 1) {
    *error = new osrmc_error{"InvalidNodes", "Nodes array is empty"};
    return 0;
  }

  const auto from_node = std::get<osrm::json::Number>(nodes.values[0]).value;
  return static_cast<unsigned long long>(from_node);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return 0;
}

unsigned long long
osrmc_nearest_response_to_node(osrmc_nearest_response_t response, unsigned index, osrmc_error_t* error) try {
  if (!osrmc_validate_nearest_response(response, error)) {
    return 0;
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  const auto& waypoints = std::get<osrm::json::Array>(response_typed->values.at("waypoints"));
  if (index >= waypoints.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Waypoint index out of bounds"};
    return 0;
  }

  const auto& waypoint = std::get<osrm::json::Object>(waypoints.values.at(index));
  const auto nodes_iter = waypoint.values.find("nodes");
  if (nodes_iter == waypoint.values.end()) {
    *error = new osrmc_error{"NoNodes", "Nodes not available for this waypoint"};
    return 0;
  }

  const auto& nodes = std::get<osrm::json::Array>(nodes_iter->second);
  if (nodes.values.size() < 2) {
    *error = new osrmc_error{"InvalidNodes", "Nodes array does not have enough elements"};
    return 0;
  }

  const auto to_node = std::get<osrm::json::Number>(nodes.values[1]).value;
  return static_cast<unsigned long long>(to_node);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return 0;
}

const char*
osrmc_nearest_response_data_version(osrmc_nearest_response_t response, osrmc_error_t* error) try {
  if (!osrmc_validate_nearest_response(response, error)) {
    return nullptr;
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  const auto data_version_iter = response_typed->values.find("data_version");
  if (data_version_iter == response_typed->values.end()) {
    return nullptr;
  }

  const auto& data_version = std::get<osrm::json::String>(data_version_iter->second).value;
  return data_version.c_str();
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

osrmc_blob_t
osrmc_nearest_response_json(osrmc_nearest_response_t response, osrmc_error_t* error) try {
  if (!osrmc_validate_nearest_response(response, error)) {
    return nullptr;
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);
  return osrmc_render_json(*response_typed);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

/* Route */

osrmc_route_params_t
osrmc_route_params_construct(osrmc_error_t* error) try {
  auto* out = new osrm::RouteParameters;

  return reinterpret_cast<osrmc_route_params_t>(out);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

void
osrmc_route_params_destruct(osrmc_route_params_t params) {
  if (params) {
    delete reinterpret_cast<osrm::RouteParameters*>(params);
  }
}

void
osrmc_route_params_set_steps(osrmc_route_params_t params, int on, osrmc_error_t* error) try {
  if (!osrmc_validate_params(reinterpret_cast<osrmc_params_t>(params), error)) {
    return;
  }
  osrmc_route_like_set_steps_flag(params, on);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_route_params_set_alternatives(osrmc_route_params_t params, int on, osrmc_error_t* error) try {
  if (!osrmc_validate_params(reinterpret_cast<osrmc_params_t>(params), error)) {
    return;
  }
  osrmc_route_like_set_alternatives_flag(params, on);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_route_params_set_geometries(osrmc_route_params_t params,
                                  osrmc_geometries_type_t geometries,
                                  osrmc_error_t* error) try {
  auto* params_typed = osrmc_route_like_params(params);
  switch (geometries) {
    case OSRMC_GEOMETRIES_POLYLINE:
      params_typed->geometries = osrm::RouteParameters::GeometriesType::Polyline;
      break;
    case OSRMC_GEOMETRIES_POLYLINE6:
      params_typed->geometries = osrm::RouteParameters::GeometriesType::Polyline6;
      break;
    case OSRMC_GEOMETRIES_GEOJSON:
      params_typed->geometries = osrm::RouteParameters::GeometriesType::GeoJSON;
      break;
    default:
      if (error) {
        *error = new osrmc_error{"InvalidArgument", "Unknown geometries type"};
      }
      return;
  }
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_route_params_set_overview(osrmc_route_params_t params, osrmc_overview_type_t overview, osrmc_error_t* error) try {
  auto* params_typed = osrmc_route_like_params(params);
  switch (overview) {
    case OSRMC_OVERVIEW_SIMPLIFIED:
      params_typed->overview = osrm::RouteParameters::OverviewType::Simplified;
      break;
    case OSRMC_OVERVIEW_FULL:
      params_typed->overview = osrm::RouteParameters::OverviewType::Full;
      break;
    case OSRMC_OVERVIEW_FALSE:
      params_typed->overview = osrm::RouteParameters::OverviewType::False;
      break;
    default:
      if (error) {
        *error = new osrmc_error{"InvalidArgument", "Unknown overview type"};
      }
      return;
  }
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_route_params_set_continue_straight(osrmc_route_params_t params, int on, osrmc_error_t* error) try {
  osrmc_route_like_set_continue_straight_impl(osrmc_route_like_params(params), on);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_route_params_set_number_of_alternatives(osrmc_route_params_t params, unsigned count, osrmc_error_t* error) try {
  osrmc_route_like_set_number_of_alternatives_value(params, count);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_route_params_set_annotations(osrmc_route_params_t params,
                                   osrmc_annotations_type_t annotations,
                                   osrmc_error_t* error) try {
  auto* params_typed = osrmc_route_like_params(params);
  params_typed->annotations_type = static_cast<osrm::RouteParameters::AnnotationsType>(annotations);
  params_typed->annotations = (annotations != OSRMC_ANNOTATIONS_NONE);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_route_params_add_waypoint(osrmc_route_params_t params, size_t index, osrmc_error_t* error) try {
  osrmc_route_like_add_waypoint_impl(osrmc_route_like_params(params), index);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_route_params_clear_waypoints(osrmc_route_params_t params) {
  osrmc_route_like_clear_waypoints_impl(osrmc_route_like_params(params));
}

osrmc_route_response_t
osrmc_route(osrmc_osrm_t osrm, osrmc_route_params_t params, osrmc_error_t* error) try {
  if (!osrmc_validate_osrm(osrm, error) || !params) {
    if (error) {
      *error = new osrmc_error{"InvalidArgument", "OSRM instance and params must not be null"};
    }
    return nullptr;
  }
  auto* osrm_typed = reinterpret_cast<osrm::OSRM*>(osrm);
  auto* params_typed = reinterpret_cast<osrm::RouteParameters*>(params);

  osrm::engine::api::ResultT result = osrm::json::Object();
  const auto status = osrm_typed->Route(*params_typed, result);

  if (status == osrm::Status::Ok) {
    auto* out = new osrmc_route_response;
    out->geometries = params_typed->geometries;

    if (std::holds_alternative<osrm::json::Object>(result)) {
      out->json = std::get<osrm::json::Object>(result);
      return reinterpret_cast<osrmc_route_response_t>(out);
    } else {
      if (error) {
        *error = new osrmc_error{"InvalidResponse", "Unexpected response type"};
      }
      delete out;
      return nullptr;
    }
  }

  if (std::holds_alternative<osrm::json::Object>(result)) {
    osrmc_error_from_json(std::get<osrm::json::Object>(result), error);
  } else {
    if (error) {
      *error = new osrmc_error{"RouteError", "Route request failed"};
    }
  }
  return nullptr;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

void
osrmc_route_response_destruct(osrmc_route_response_t response) {
  if (response) {
    delete osrmc_get_route_response(response);
  }
}

double
osrmc_route_response_distance(osrmc_route_response_t response, osrmc_error_t* error) try {
  if (!osrmc_validate_route_response(response, error)) {
    return std::numeric_limits<double>::infinity();
  }
  auto* response_holder = osrmc_get_route_response(response);
  auto& response_json = response_holder->json;

  auto& routes = std::get<osrm::json::Array>(response_json.values["routes"]);
  auto& route = std::get<osrm::json::Object>(routes.values.at(0));

  const auto distance = std::get<osrm::json::Number>(route.values["distance"]).value;
  return distance;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::infinity();
}

double
osrmc_route_response_duration(osrmc_route_response_t response, osrmc_error_t* error) try {
  if (!osrmc_validate_route_response(response, error)) {
    return std::numeric_limits<double>::infinity();
  }
  auto* response_holder = osrmc_get_route_response(response);
  auto& response_json = response_holder->json;

  auto& routes = std::get<osrm::json::Array>(response_json.values["routes"]);
  auto& route = std::get<osrm::json::Object>(routes.values.at(0));

  const auto duration = std::get<osrm::json::Number>(route.values["duration"]).value;
  return duration;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::infinity();
}

double
osrmc_route_response_weight(osrmc_route_response_t response, osrmc_error_t* error) try {
  if (!osrmc_validate_route_response(response, error)) {
    return std::numeric_limits<double>::infinity();
  }
  auto* response_holder = osrmc_get_route_response(response);
  auto& response_json = response_holder->json;

  auto& routes = std::get<osrm::json::Array>(response_json.values["routes"]);
  auto& route = std::get<osrm::json::Object>(routes.values.at(0));

  const auto weight_iter = route.values.find("weight");
  if (weight_iter == route.values.end()) {
    *error = new osrmc_error{"NoWeight", "Weight not available for this route"};
    return std::numeric_limits<double>::infinity();
  }

  const auto weight = std::get<osrm::json::Number>(weight_iter->second).value;
  return weight;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::infinity();
}

const char*
osrmc_route_response_weight_name(osrmc_route_response_t response, osrmc_error_t* error) try {
  if (!osrmc_validate_route_response(response, error)) {
    return nullptr;
  }
  auto* response_holder = osrmc_get_route_response(response);
  auto& response_json = response_holder->json;

  auto& routes = std::get<osrm::json::Array>(response_json.values["routes"]);
  auto& route = std::get<osrm::json::Object>(routes.values.at(0));

  const auto weight_name_iter = route.values.find("weight_name");
  if (weight_name_iter == route.values.end()) {
    *error = new osrmc_error{"NoWeightName", "Weight name not available for this route"};
    return nullptr;
  }

  const auto& weight_name = std::get<osrm::json::String>(weight_name_iter->second).value;
  return weight_name.c_str();
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

unsigned
osrmc_route_response_alternative_count(osrmc_route_response_t response, osrmc_error_t* error) try {
  if (!osrmc_validate_route_response(response, error)) {
    return 0;
  }
  auto* response_holder = osrmc_get_route_response(response);
  auto& response_json = response_holder->json;

  if (response_json.values.find("routes") == response_json.values.end()) {
    return 0;
  }

  const auto& routes = std::get<osrm::json::Array>(response_json.values.at("routes"));
  return static_cast<unsigned>(routes.values.size());
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return 0;
}

double
osrmc_route_response_distance_at(osrmc_route_response_t response, unsigned route_index, osrmc_error_t* error) try {
  if (!osrmc_validate_route_response(response, error)) {
    return std::numeric_limits<double>::infinity();
  }
  auto* response_holder = osrmc_get_route_response(response);
  auto& response_json = response_holder->json;

  auto& routes = std::get<osrm::json::Array>(response_json.values["routes"]);
  if (route_index >= routes.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Route index out of bounds"};
    return std::numeric_limits<double>::infinity();
  }

  auto& route = std::get<osrm::json::Object>(routes.values.at(route_index));
  const auto distance = std::get<osrm::json::Number>(route.values["distance"]).value;
  return distance;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::infinity();
}

double
osrmc_route_response_duration_at(osrmc_route_response_t response, unsigned route_index, osrmc_error_t* error) try {
  if (!osrmc_validate_route_response(response, error)) {
    return std::numeric_limits<double>::infinity();
  }
  auto* response_holder = osrmc_get_route_response(response);
  auto& response_json = response_holder->json;

  auto& routes = std::get<osrm::json::Array>(response_json.values["routes"]);
  if (route_index >= routes.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Route index out of bounds"};
    return std::numeric_limits<double>::infinity();
  }

  auto& route = std::get<osrm::json::Object>(routes.values.at(route_index));
  const auto duration = std::get<osrm::json::Number>(route.values["duration"]).value;
  return duration;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::infinity();
}

double
osrmc_route_response_weight_at(osrmc_route_response_t response, unsigned route_index, osrmc_error_t* error) try {
  if (!osrmc_validate_route_response(response, error)) {
    return std::numeric_limits<double>::infinity();
  }
  auto* response_holder = osrmc_get_route_response(response);
  auto& response_json = response_holder->json;

  auto& routes = std::get<osrm::json::Array>(response_json.values["routes"]);
  if (route_index >= routes.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Route index out of bounds"};
    return std::numeric_limits<double>::infinity();
  }

  auto& route = std::get<osrm::json::Object>(routes.values.at(route_index));
  const auto weight_iter = route.values.find("weight");
  if (weight_iter == route.values.end()) {
    *error = new osrmc_error{"NoWeight", "Weight not available for this route"};
    return std::numeric_limits<double>::infinity();
  }

  const auto weight = std::get<osrm::json::Number>(weight_iter->second).value;
  return weight;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::infinity();
}

const char*
osrmc_route_response_geometry_polyline(osrmc_route_response_t response,
                                       unsigned route_index,
                                       osrmc_error_t* error) try {
  if (!osrmc_validate_route_response(response, error)) {
    return nullptr;
  }
  auto* response_holder = osrmc_get_route_response(response);
  auto& response_json = response_holder->json;
  auto& routes = std::get<osrm::json::Array>(response_json.values["routes"]);
  if (route_index >= routes.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Route index out of bounds"};
    return nullptr;
  }

  auto& route = std::get<osrm::json::Object>(routes.values.at(route_index));
  const auto geometry_iter = route.values.find("geometry");
  if (geometry_iter == route.values.end()) {
    *error = new osrmc_error{"NoGeometry", "Geometry not available for this route"};
    return nullptr;
  }

  const auto& geometry = geometry_iter->second;
  if (std::holds_alternative<osrm::json::String>(geometry)) {
    const auto& polyline = std::get<osrm::json::String>(geometry).value;
    return polyline.c_str();
  }

  *error = new osrmc_error{"NoPolyline", "Geometry not available for this route"};
  return nullptr;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

unsigned
osrmc_route_response_geometry_coordinate_count(osrmc_route_response_t response,
                                               unsigned route_index,
                                               osrmc_error_t* error) try {
  if (!osrmc_validate_route_response(response, error)) {
    return 0;
  }
  auto* response_holder = osrmc_get_route_response(response);
  auto& response_json = response_holder->json;

  auto& routes = std::get<osrm::json::Array>(response_json.values["routes"]);
  if (route_index >= routes.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Route index out of bounds"};
    return 0;
  }

  auto& route = std::get<osrm::json::Object>(routes.values.at(route_index));
  const auto* coordinates = osrmc_get_route_geometry_coordinates(route, error);
  if (!coordinates) {
    return 0;
  }
  return static_cast<unsigned>(coordinates->values.size());
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return 0;
}

double
osrmc_route_response_geometry_coordinate_latitude(osrmc_route_response_t response,
                                                  unsigned route_index,
                                                  unsigned coord_index,
                                                  osrmc_error_t* error) try {
  auto* response_holder = osrmc_get_route_response(response);
  auto& response_json = response_holder->json;

  auto& routes = std::get<osrm::json::Array>(response_json.values["routes"]);
  if (route_index >= routes.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Route index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  auto& route = std::get<osrm::json::Object>(routes.values.at(route_index));
  const auto* coordinates = osrmc_get_route_geometry_coordinates(route, error);
  if (!coordinates) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  if (coord_index >= coordinates->values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Coordinate index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& coordinate_pair = std::get<osrm::json::Array>(coordinates->values.at(coord_index)).values;
  if (coordinate_pair.size() < MIN_COORDINATE_PAIR_SIZE) {
    *error = new osrmc_error{"InvalidGeometry", "Coordinate entry is malformed"};
    return std::numeric_limits<double>::quiet_NaN();
  }
  return std::get<osrm::json::Number>(coordinate_pair[COORDINATE_LATITUDE_INDEX]).value;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::quiet_NaN();
}

double
osrmc_route_response_geometry_coordinate_longitude(osrmc_route_response_t response,
                                                   unsigned route_index,
                                                   unsigned coord_index,
                                                   osrmc_error_t* error) try {
  if (!osrmc_validate_route_response(response, error)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  auto* response_holder = osrmc_get_route_response(response);
  auto& response_json = response_holder->json;

  auto& routes = std::get<osrm::json::Array>(response_json.values["routes"]);
  if (route_index >= routes.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Route index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  auto& route = std::get<osrm::json::Object>(routes.values.at(route_index));
  const auto* coordinates = osrmc_get_route_geometry_coordinates(route, error);
  if (!coordinates) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  if (coord_index >= coordinates->values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Coordinate index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& coordinate_pair = std::get<osrm::json::Array>(coordinates->values.at(coord_index)).values;
  if (coordinate_pair.size() < MIN_COORDINATE_PAIR_SIZE) {
    *error = new osrmc_error{"InvalidGeometry", "Coordinate entry is malformed"};
    return std::numeric_limits<double>::quiet_NaN();
  }
  return std::get<osrm::json::Number>(coordinate_pair[COORDINATE_LONGITUDE_INDEX]).value;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::quiet_NaN();
}

unsigned
osrmc_route_response_waypoint_count(osrmc_route_response_t response, osrmc_error_t* error) try {
  if (!osrmc_validate_route_response(response, error)) {
    return 0;
  }
  auto* response_holder = osrmc_get_route_response(response);
  auto& response_json = response_holder->json;

  if (response_json.values.find("waypoints") == response_json.values.end()) {
    return 0;
  }

  const auto& waypoints = std::get<osrm::json::Array>(response_json.values.at("waypoints"));
  return static_cast<unsigned>(waypoints.values.size());
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return 0;
}

double
osrmc_route_response_waypoint_latitude(osrmc_route_response_t response, unsigned index, osrmc_error_t* error) try {
  if (!osrmc_validate_route_response(response, error)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  auto* response_holder = osrmc_get_route_response(response);
  auto& response_json = response_holder->json;

  const auto& waypoints = std::get<osrm::json::Array>(response_json.values.at("waypoints"));
  if (index >= waypoints.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Waypoint index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& waypoint = std::get<osrm::json::Object>(waypoints.values.at(index));
  const auto& location = std::get<osrm::json::Array>(waypoint.values.at("location")).values;
  const auto latitude = std::get<osrm::json::Number>(location[COORDINATE_LATITUDE_INDEX]).value;

  return latitude;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::quiet_NaN();
}

double
osrmc_route_response_waypoint_longitude(osrmc_route_response_t response, unsigned index, osrmc_error_t* error) try {
  if (!osrmc_validate_route_response(response, error)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  auto* response_holder = osrmc_get_route_response(response);
  auto& response_json = response_holder->json;

  const auto& waypoints = std::get<osrm::json::Array>(response_json.values.at("waypoints"));
  if (index >= waypoints.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Waypoint index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& waypoint = std::get<osrm::json::Object>(waypoints.values.at(index));
  const auto& location = std::get<osrm::json::Array>(waypoint.values.at("location")).values;
  const auto longitude = std::get<osrm::json::Number>(location[COORDINATE_LONGITUDE_INDEX]).value;

  return longitude;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::quiet_NaN();
}

const char*
osrmc_route_response_waypoint_name(osrmc_route_response_t response, unsigned index, osrmc_error_t* error) try {
  if (!osrmc_validate_route_response(response, error)) {
    return nullptr;
  }
  auto* response_holder = osrmc_get_route_response(response);
  auto& response_json = response_holder->json;

  const auto& waypoints = std::get<osrm::json::Array>(response_json.values.at("waypoints"));
  if (index >= waypoints.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Waypoint index out of bounds"};
    return nullptr;
  }

  const auto& waypoint = std::get<osrm::json::Object>(waypoints.values.at(index));
  if (waypoint.values.find("name") == waypoint.values.end()) {
    return nullptr;
  }

  const auto& name = std::get<osrm::json::String>(waypoint.values.at("name")).value;
  return name.c_str();
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

unsigned
osrmc_route_response_leg_count(osrmc_route_response_t response, unsigned route_index, osrmc_error_t* error) try {
  if (!osrmc_validate_route_response(response, error)) {
    return 0;
  }
  auto* response_holder = osrmc_get_route_response(response);
  auto& response_json = response_holder->json;

  auto& routes = std::get<osrm::json::Array>(response_json.values["routes"]);
  if (route_index >= routes.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Route index out of bounds"};
    return 0;
  }

  auto& route = std::get<osrm::json::Object>(routes.values.at(route_index));
  if (route.values.find("legs") == route.values.end()) {
    return 0;
  }

  const auto& legs = std::get<osrm::json::Array>(route.values.at("legs"));
  return static_cast<unsigned>(legs.values.size());
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return 0;
}

double
osrmc_route_response_leg_distance(osrmc_route_response_t response,
                                  unsigned route_index,
                                  unsigned leg_index,
                                  osrmc_error_t* error) try {
  if (!osrmc_validate_route_response(response, error)) {
    return std::numeric_limits<double>::infinity();
  }
  auto* response_holder = osrmc_get_route_response(response);
  const auto* leg = osrmc_get_route_leg(response_holder->json, route_index, leg_index, error);
  if (!leg) {
    return std::numeric_limits<double>::infinity();
  }

  const auto distance_iter = leg->values.find("distance");
  if (distance_iter == leg->values.end()) {
    if (error) {
      *error = new osrmc_error{"NoDistance", "Distance not available for this leg"};
    }
    return std::numeric_limits<double>::infinity();
  }

  const auto distance = std::get<osrm::json::Number>(distance_iter->second).value;
  return distance;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::infinity();
}

double
osrmc_route_response_leg_duration(osrmc_route_response_t response,
                                  unsigned route_index,
                                  unsigned leg_index,
                                  osrmc_error_t* error) try {
  if (!osrmc_validate_route_response(response, error)) {
    return std::numeric_limits<double>::infinity();
  }
  auto* response_holder = osrmc_get_route_response(response);
  const auto* leg = osrmc_get_route_leg(response_holder->json, route_index, leg_index, error);
  if (!leg) {
    return std::numeric_limits<double>::infinity();
  }

  const auto duration_iter = leg->values.find("duration");
  if (duration_iter == leg->values.end()) {
    if (error) {
      *error = new osrmc_error{"NoDuration", "Duration not available for this leg"};
    }
    return std::numeric_limits<double>::infinity();
  }

  const auto duration = std::get<osrm::json::Number>(duration_iter->second).value;
  return duration;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::infinity();
}

double
osrmc_route_response_leg_weight(osrmc_route_response_t response,
                                unsigned route_index,
                                unsigned leg_index,
                                osrmc_error_t* error) try {
  if (!osrmc_validate_route_response(response, error)) {
    return std::numeric_limits<double>::infinity();
  }
  auto* response_holder = osrmc_get_route_response(response);
  auto& response_json = response_holder->json;

  auto& routes = std::get<osrm::json::Array>(response_json.values["routes"]);
  if (route_index >= routes.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Route index out of bounds"};
    return std::numeric_limits<double>::infinity();
  }

  auto& route = std::get<osrm::json::Object>(routes.values.at(route_index));
  const auto& legs = std::get<osrm::json::Array>(route.values.at("legs"));
  if (leg_index >= legs.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Leg index out of bounds"};
    return std::numeric_limits<double>::infinity();
  }

  const auto& leg = std::get<osrm::json::Object>(legs.values.at(leg_index));
  const auto weight_iter = leg.values.find("weight");
  if (weight_iter == leg.values.end()) {
    *error = new osrmc_error{"NoWeight", "Weight not available for this leg"};
    return std::numeric_limits<double>::infinity();
  }

  const auto weight = std::get<osrm::json::Number>(weight_iter->second).value;
  return weight;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::infinity();
}

const char*
osrmc_route_response_leg_summary(osrmc_route_response_t response,
                                 unsigned route_index,
                                 unsigned leg_index,
                                 osrmc_error_t* error) try {
  if (!osrmc_validate_route_response(response, error)) {
    return nullptr;
  }
  auto* response_holder = osrmc_get_route_response(response);
  const auto* leg = osrmc_get_route_leg(response_holder->json, route_index, leg_index, error);
  if (!leg) {
    return nullptr;
  }

  const auto summary_iter = leg->values.find("summary");
  if (summary_iter == leg->values.end()) {
    return nullptr;
  }

  const auto& summary = std::get<osrm::json::String>(summary_iter->second).value;
  return summary.c_str();
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

unsigned
osrmc_route_response_leg_annotations_count(osrmc_route_response_t response,
                                           unsigned route_index,
                                           unsigned leg_index,
                                           osrmc_error_t* error) try {
  if (!osrmc_validate_route_response(response, error)) {
    return 0;
  }
  auto* response_holder = osrmc_get_route_response(response);
  auto& response_json = response_holder->json;

  auto& routes = std::get<osrm::json::Array>(response_json.values["routes"]);
  if (route_index >= routes.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Route index out of bounds"};
    return 0;
  }

  auto& route = std::get<osrm::json::Object>(routes.values.at(route_index));
  const auto& legs = std::get<osrm::json::Array>(route.values.at("legs"));
  if (leg_index >= legs.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Leg index out of bounds"};
    return 0;
  }

  const auto& leg = std::get<osrm::json::Object>(legs.values.at(leg_index));
  const auto annotations_iter = leg.values.find("annotation");
  if (annotations_iter == leg.values.end()) {
    return 0;
  }

  const auto& annotation = std::get<osrm::json::Object>(annotations_iter->second);

  if (annotation.values.find("duration") != annotation.values.end()) {
    const auto& duration_array = std::get<osrm::json::Array>(annotation.values.at("duration"));
    return static_cast<unsigned>(duration_array.values.size());
  }
  if (annotation.values.find("distance") != annotation.values.end()) {
    const auto& distance_array = std::get<osrm::json::Array>(annotation.values.at("distance"));
    return static_cast<unsigned>(distance_array.values.size());
  }
  if (annotation.values.find("speed") != annotation.values.end()) {
    const auto& speed_array = std::get<osrm::json::Array>(annotation.values.at("speed"));
    return static_cast<unsigned>(speed_array.values.size());
  }
  if (annotation.values.find("weight") != annotation.values.end()) {
    const auto& weight_array = std::get<osrm::json::Array>(annotation.values.at("weight"));
    return static_cast<unsigned>(weight_array.values.size());
  }
  if (annotation.values.find("nodes") != annotation.values.end()) {
    const auto& nodes_array = std::get<osrm::json::Array>(annotation.values.at("nodes"));
    return static_cast<unsigned>(nodes_array.values.size());
  }
  if (annotation.values.find("datasources") != annotation.values.end()) {
    const auto& datasources_array = std::get<osrm::json::Array>(annotation.values.at("datasources"));
    return static_cast<unsigned>(datasources_array.values.size());
  }

  return 0;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return 0;
}

double
osrmc_route_response_leg_annotations_speed(osrmc_route_response_t response,
                                           unsigned route_index,
                                           unsigned leg_index,
                                           unsigned annotation_index,
                                           osrmc_error_t* error) try {
  if (!osrmc_validate_route_response(response, error)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  auto* response_holder = osrmc_get_route_response(response);
  auto& response_json = response_holder->json;

  auto& routes = std::get<osrm::json::Array>(response_json.values["routes"]);
  if (route_index >= routes.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Route index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  auto& route = std::get<osrm::json::Object>(routes.values.at(route_index));
  const auto& legs = std::get<osrm::json::Array>(route.values.at("legs"));
  if (leg_index >= legs.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Leg index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& leg = std::get<osrm::json::Object>(legs.values.at(leg_index));
  const auto annotations_iter = leg.values.find("annotation");
  if (annotations_iter == leg.values.end()) {
    *error = new osrmc_error{"NoAnnotations", "Annotations not available for this leg"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& annotation = std::get<osrm::json::Object>(annotations_iter->second);
  const auto speed_iter = annotation.values.find("speed");
  if (speed_iter == annotation.values.end()) {
    *error = new osrmc_error{"NoSpeed", "Speed annotations not available"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& speed_array = std::get<osrm::json::Array>(speed_iter->second);
  if (annotation_index >= speed_array.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Annotation index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto speed = std::get<osrm::json::Number>(speed_array.values.at(annotation_index)).value;
  return speed;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::quiet_NaN();
}

double
osrmc_route_response_leg_annotations_duration(osrmc_route_response_t response,
                                              unsigned route_index,
                                              unsigned leg_index,
                                              unsigned annotation_index,
                                              osrmc_error_t* error) try {
  if (!osrmc_validate_route_response(response, error)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  auto* response_holder = osrmc_get_route_response(response);
  auto& response_json = response_holder->json;

  auto& routes = std::get<osrm::json::Array>(response_json.values["routes"]);
  if (route_index >= routes.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Route index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  auto& route = std::get<osrm::json::Object>(routes.values.at(route_index));
  const auto& legs = std::get<osrm::json::Array>(route.values.at("legs"));
  if (leg_index >= legs.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Leg index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& leg = std::get<osrm::json::Object>(legs.values.at(leg_index));
  const auto annotations_iter = leg.values.find("annotation");
  if (annotations_iter == leg.values.end()) {
    *error = new osrmc_error{"NoAnnotations", "Annotations not available for this leg"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& annotation = std::get<osrm::json::Object>(annotations_iter->second);
  const auto duration_iter = annotation.values.find("duration");
  if (duration_iter == annotation.values.end()) {
    *error = new osrmc_error{"NoDuration", "Duration annotations not available"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& duration_array = std::get<osrm::json::Array>(duration_iter->second);
  if (annotation_index >= duration_array.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Annotation index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto duration = std::get<osrm::json::Number>(duration_array.values.at(annotation_index)).value;
  return duration;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::quiet_NaN();
}

double
osrmc_route_response_leg_annotations_distance(osrmc_route_response_t response,
                                              unsigned route_index,
                                              unsigned leg_index,
                                              unsigned annotation_index,
                                              osrmc_error_t* error) try {
  if (!osrmc_validate_route_response(response, error)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  auto* response_holder = osrmc_get_route_response(response);
  auto& response_json = response_holder->json;

  auto& routes = std::get<osrm::json::Array>(response_json.values["routes"]);
  if (route_index >= routes.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Route index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  auto& route = std::get<osrm::json::Object>(routes.values.at(route_index));
  const auto& legs = std::get<osrm::json::Array>(route.values.at("legs"));
  if (leg_index >= legs.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Leg index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& leg = std::get<osrm::json::Object>(legs.values.at(leg_index));
  const auto annotations_iter = leg.values.find("annotation");
  if (annotations_iter == leg.values.end()) {
    *error = new osrmc_error{"NoAnnotations", "Annotations not available for this leg"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& annotation = std::get<osrm::json::Object>(annotations_iter->second);
  const auto distance_iter = annotation.values.find("distance");
  if (distance_iter == annotation.values.end()) {
    *error = new osrmc_error{"NoDistance", "Distance annotations not available"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& distance_array = std::get<osrm::json::Array>(distance_iter->second);
  if (annotation_index >= distance_array.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Annotation index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto distance = std::get<osrm::json::Number>(distance_array.values.at(annotation_index)).value;
  return distance;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::quiet_NaN();
}

double
osrmc_route_response_leg_annotations_weight(osrmc_route_response_t response,
                                            unsigned route_index,
                                            unsigned leg_index,
                                            unsigned annotation_index,
                                            osrmc_error_t* error) try {
  if (!osrmc_validate_route_response(response, error)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  auto* response_holder = osrmc_get_route_response(response);
  auto& response_json = response_holder->json;

  auto& routes = std::get<osrm::json::Array>(response_json.values["routes"]);
  if (route_index >= routes.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Route index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  auto& route = std::get<osrm::json::Object>(routes.values.at(route_index));
  const auto& legs = std::get<osrm::json::Array>(route.values.at("legs"));
  if (leg_index >= legs.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Leg index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& leg = std::get<osrm::json::Object>(legs.values.at(leg_index));
  const auto annotations_iter = leg.values.find("annotation");
  if (annotations_iter == leg.values.end()) {
    *error = new osrmc_error{"NoAnnotations", "Annotations not available for this leg"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& annotation = std::get<osrm::json::Object>(annotations_iter->second);
  const auto weight_iter = annotation.values.find("weight");
  if (weight_iter == annotation.values.end()) {
    *error = new osrmc_error{"NoWeight", "Weight annotations not available"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& weight_array = std::get<osrm::json::Array>(weight_iter->second);
  if (annotation_index >= weight_array.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Annotation index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto weight = std::get<osrm::json::Number>(weight_array.values.at(annotation_index)).value;
  return weight;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::quiet_NaN();
}

unsigned
osrmc_route_response_leg_annotations_datasources(osrmc_route_response_t response,
                                                 unsigned route_index,
                                                 unsigned leg_index,
                                                 unsigned annotation_index,
                                                 osrmc_error_t* error) try {
  if (!osrmc_validate_route_response(response, error)) {
    return 0;
  }
  auto* response_holder = osrmc_get_route_response(response);
  auto& response_json = response_holder->json;

  auto& routes = std::get<osrm::json::Array>(response_json.values["routes"]);
  if (route_index >= routes.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Route index out of bounds"};
    return 0;
  }

  auto& route = std::get<osrm::json::Object>(routes.values.at(route_index));
  const auto& legs = std::get<osrm::json::Array>(route.values.at("legs"));
  if (leg_index >= legs.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Leg index out of bounds"};
    return 0;
  }

  const auto& leg = std::get<osrm::json::Object>(legs.values.at(leg_index));
  const auto annotations_iter = leg.values.find("annotation");
  if (annotations_iter == leg.values.end()) {
    *error = new osrmc_error{"NoAnnotations", "Annotations not available for this leg"};
    return 0;
  }

  const auto& annotation = std::get<osrm::json::Object>(annotations_iter->second);
  const auto datasources_iter = annotation.values.find("datasources");
  if (datasources_iter == annotation.values.end()) {
    *error = new osrmc_error{"NoDatasources", "Datasources annotations not available"};
    return 0;
  }

  const auto& datasources_array = std::get<osrm::json::Array>(datasources_iter->second);
  if (annotation_index >= datasources_array.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Annotation index out of bounds"};
    return 0;
  }

  const auto datasource = std::get<osrm::json::Number>(datasources_array.values.at(annotation_index)).value;
  return static_cast<unsigned>(datasource);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return 0;
}

unsigned long long
osrmc_route_response_leg_annotations_nodes(osrmc_route_response_t response,
                                           unsigned route_index,
                                           unsigned leg_index,
                                           unsigned annotation_index,
                                           osrmc_error_t* error) try {
  auto* response_holder = osrmc_get_route_response(response);
  auto& response_json = response_holder->json;

  auto& routes = std::get<osrm::json::Array>(response_json.values["routes"]);
  if (route_index >= routes.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Route index out of bounds"};
    return 0;
  }

  auto& route = std::get<osrm::json::Object>(routes.values.at(route_index));
  const auto& legs = std::get<osrm::json::Array>(route.values.at("legs"));
  if (leg_index >= legs.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Leg index out of bounds"};
    return 0;
  }

  const auto& leg = std::get<osrm::json::Object>(legs.values.at(leg_index));
  const auto annotations_iter = leg.values.find("annotation");
  if (annotations_iter == leg.values.end()) {
    *error = new osrmc_error{"NoAnnotations", "Annotations not available for this leg"};
    return 0;
  }

  const auto& annotation = std::get<osrm::json::Object>(annotations_iter->second);
  const auto nodes_iter = annotation.values.find("nodes");
  if (nodes_iter == annotation.values.end()) {
    *error = new osrmc_error{"NoNodes", "Nodes annotations not available"};
    return 0;
  }

  const auto& nodes_array = std::get<osrm::json::Array>(nodes_iter->second);
  if (annotation_index >= nodes_array.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Annotation index out of bounds"};
    return 0;
  }

  const auto node = std::get<osrm::json::Number>(nodes_array.values.at(annotation_index)).value;
  return static_cast<unsigned long long>(node);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return 0;
}

unsigned
osrmc_route_response_step_count(osrmc_route_response_t response,
                                unsigned route_index,
                                unsigned leg_index,
                                osrmc_error_t* error) try {
  if (!osrmc_validate_route_response(response, error)) {
    return 0;
  }
  auto* response_holder = osrmc_get_route_response(response);
  auto& response_json = response_holder->json;

  auto& routes = std::get<osrm::json::Array>(response_json.values["routes"]);
  if (route_index >= routes.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Route index out of bounds"};
    return 0;
  }

  auto& route = std::get<osrm::json::Object>(routes.values.at(route_index));
  const auto& legs = std::get<osrm::json::Array>(route.values.at("legs"));
  if (leg_index >= legs.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Leg index out of bounds"};
    return 0;
  }

  const auto& leg = std::get<osrm::json::Object>(legs.values.at(leg_index));
  if (leg.values.find("steps") == leg.values.end()) {
    return 0;
  }

  const auto& steps = std::get<osrm::json::Array>(leg.values.at("steps"));
  return static_cast<unsigned>(steps.values.size());
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return 0;
}

double
osrmc_route_response_step_distance(osrmc_route_response_t response,
                                   unsigned route_index,
                                   unsigned leg_index,
                                   unsigned step_index,
                                   osrmc_error_t* error) try {
  auto* response_holder = osrmc_get_route_response(response);
  auto& response_json = response_holder->json;

  auto& routes = std::get<osrm::json::Array>(response_json.values["routes"]);
  if (route_index >= routes.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Route index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  auto& route = std::get<osrm::json::Object>(routes.values.at(route_index));
  const auto& legs = std::get<osrm::json::Array>(route.values.at("legs"));
  if (leg_index >= legs.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Leg index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& leg = std::get<osrm::json::Object>(legs.values.at(leg_index));
  const auto& steps = std::get<osrm::json::Array>(leg.values.at("steps"));
  if (step_index >= steps.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Step index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& step = std::get<osrm::json::Object>(steps.values.at(step_index));
  const auto distance = std::get<osrm::json::Number>(step.values.at("distance")).value;

  return distance;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::quiet_NaN();
}

double
osrmc_route_response_step_duration(osrmc_route_response_t response,
                                   unsigned route_index,
                                   unsigned leg_index,
                                   unsigned step_index,
                                   osrmc_error_t* error) try {
  if (!osrmc_validate_route_response(response, error)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  auto* response_holder = osrmc_get_route_response(response);
  auto& response_json = response_holder->json;

  auto& routes = std::get<osrm::json::Array>(response_json.values["routes"]);
  if (route_index >= routes.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Route index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  auto& route = std::get<osrm::json::Object>(routes.values.at(route_index));
  const auto& legs = std::get<osrm::json::Array>(route.values.at("legs"));
  if (leg_index >= legs.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Leg index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& leg = std::get<osrm::json::Object>(legs.values.at(leg_index));
  const auto& steps = std::get<osrm::json::Array>(leg.values.at("steps"));
  if (step_index >= steps.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Step index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& step = std::get<osrm::json::Object>(steps.values.at(step_index));
  const auto duration = std::get<osrm::json::Number>(step.values.at("duration")).value;

  return duration;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::quiet_NaN();
}

double
osrmc_route_response_step_weight(osrmc_route_response_t response,
                                 unsigned route_index,
                                 unsigned leg_index,
                                 unsigned step_index,
                                 osrmc_error_t* error) try {
  auto* response_holder = osrmc_get_route_response(response);
  auto& response_json = response_holder->json;

  auto& routes = std::get<osrm::json::Array>(response_json.values["routes"]);
  if (route_index >= routes.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Route index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  auto& route = std::get<osrm::json::Object>(routes.values.at(route_index));
  const auto& legs = std::get<osrm::json::Array>(route.values.at("legs"));
  if (leg_index >= legs.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Leg index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& leg = std::get<osrm::json::Object>(legs.values.at(leg_index));
  const auto& steps = std::get<osrm::json::Array>(leg.values.at("steps"));
  if (step_index >= steps.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Step index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& step = std::get<osrm::json::Object>(steps.values.at(step_index));
  const auto weight_iter = step.values.find("weight");
  if (weight_iter == step.values.end()) {
    *error = new osrmc_error{"NoWeight", "Weight not available for this step"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto weight = std::get<osrm::json::Number>(weight_iter->second).value;
  return weight;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::quiet_NaN();
}

const char*
osrmc_route_response_step_name(osrmc_route_response_t response,
                               unsigned route_index,
                               unsigned leg_index,
                               unsigned step_index,
                               osrmc_error_t* error) try {
  if (!osrmc_validate_route_response(response, error)) {
    return nullptr;
  }
  auto* response_holder = osrmc_get_route_response(response);
  const auto* step = osrmc_get_route_step(response_holder->json, route_index, leg_index, step_index, error);
  if (!step) {
    return nullptr;
  }

  const auto name_iter = step->values.find("name");
  if (name_iter == step->values.end()) {
    if (error) {
      *error = new osrmc_error{"NoName", "Name not available for this step"};
    }
    return nullptr;
  }

  const auto& name = std::get<osrm::json::String>(name_iter->second).value;
  return name.c_str();
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

const char*
osrmc_route_response_step_ref(osrmc_route_response_t response,
                              unsigned route_index,
                              unsigned leg_index,
                              unsigned step_index,
                              osrmc_error_t* error) try {
  if (!osrmc_validate_route_response(response, error)) {
    return nullptr;
  }
  auto* response_holder = osrmc_get_route_response(response);
  const auto* step = osrmc_get_route_step(response_holder->json, route_index, leg_index, step_index, error);
  if (!step) {
    return nullptr;
  }

  const auto ref_iter = step->values.find("ref");
  if (ref_iter == step->values.end()) {
    return nullptr;
  }

  const auto& ref = std::get<osrm::json::String>(ref_iter->second).value;
  return ref.c_str();
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

const char*
osrmc_route_response_step_pronunciation(osrmc_route_response_t response,
                                        unsigned route_index,
                                        unsigned leg_index,
                                        unsigned step_index,
                                        osrmc_error_t* error) try {
  if (!osrmc_validate_route_response(response, error)) {
    return nullptr;
  }
  auto* response_holder = osrmc_get_route_response(response);
  const auto* step = osrmc_get_route_step(response_holder->json, route_index, leg_index, step_index, error);
  if (!step) {
    return nullptr;
  }

  const auto pronunciation_iter = step->values.find("pronunciation");
  if (pronunciation_iter == step->values.end()) {
    return nullptr;
  }

  const auto& pronunciation = std::get<osrm::json::String>(pronunciation_iter->second).value;
  return pronunciation.c_str();
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

const char*
osrmc_route_response_step_destinations(osrmc_route_response_t response,
                                       unsigned route_index,
                                       unsigned leg_index,
                                       unsigned step_index,
                                       osrmc_error_t* error) try {
  if (!osrmc_validate_route_response(response, error)) {
    return nullptr;
  }
  auto* response_holder = osrmc_get_route_response(response);
  const auto* step = osrmc_get_route_step(response_holder->json, route_index, leg_index, step_index, error);
  if (!step) {
    return nullptr;
  }

  const auto destinations_iter = step->values.find("destinations");
  if (destinations_iter == step->values.end()) {
    return nullptr;
  }

  const auto& destinations = std::get<osrm::json::String>(destinations_iter->second).value;
  return destinations.c_str();
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

const char*
osrmc_route_response_step_exits(osrmc_route_response_t response,
                                unsigned route_index,
                                unsigned leg_index,
                                unsigned step_index,
                                osrmc_error_t* error) try {
  if (!osrmc_validate_route_response(response, error)) {
    return nullptr;
  }
  auto* response_holder = osrmc_get_route_response(response);
  const auto* step = osrmc_get_route_step(response_holder->json, route_index, leg_index, step_index, error);
  if (!step) {
    return nullptr;
  }

  const auto exits_iter = step->values.find("exits");
  if (exits_iter == step->values.end()) {
    return nullptr;
  }

  const auto& exits = std::get<osrm::json::String>(exits_iter->second).value;
  return exits.c_str();
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

const char*
osrmc_route_response_step_rotary_name(osrmc_route_response_t response,
                                      unsigned route_index,
                                      unsigned leg_index,
                                      unsigned step_index,
                                      osrmc_error_t* error) try {
  if (!osrmc_validate_route_response(response, error)) {
    return nullptr;
  }
  auto* response_holder = osrmc_get_route_response(response);
  const auto* step = osrmc_get_route_step(response_holder->json, route_index, leg_index, step_index, error);
  if (!step) {
    return nullptr;
  }

  const auto rotary_name_iter = step->values.find("rotary_name");
  if (rotary_name_iter == step->values.end()) {
    return nullptr;
  }

  const auto& rotary_name = std::get<osrm::json::String>(rotary_name_iter->second).value;
  return rotary_name.c_str();
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

const char*
osrmc_route_response_step_rotary_pronunciation(osrmc_route_response_t response,
                                               unsigned route_index,
                                               unsigned leg_index,
                                               unsigned step_index,
                                               osrmc_error_t* error) try {
  if (!osrmc_validate_route_response(response, error)) {
    return nullptr;
  }
  auto* response_holder = osrmc_get_route_response(response);
  const auto* step = osrmc_get_route_step(response_holder->json, route_index, leg_index, step_index, error);
  if (!step) {
    return nullptr;
  }

  const auto rotary_pronunciation_iter = step->values.find("rotary_pronunciation");
  if (rotary_pronunciation_iter == step->values.end()) {
    return nullptr;
  }

  const auto& rotary_pronunciation = std::get<osrm::json::String>(rotary_pronunciation_iter->second).value;
  return rotary_pronunciation.c_str();
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

const char*
osrmc_route_response_step_mode(osrmc_route_response_t response,
                               unsigned route_index,
                               unsigned leg_index,
                               unsigned step_index,
                               osrmc_error_t* error) try {
  if (!osrmc_validate_route_response(response, error)) {
    return nullptr;
  }
  auto* response_holder = osrmc_get_route_response(response);
  const auto* step = osrmc_get_route_step(response_holder->json, route_index, leg_index, step_index, error);
  if (!step) {
    return nullptr;
  }

  const auto mode_iter = step->values.find("mode");
  if (mode_iter == step->values.end()) {
    if (error) {
      *error = new osrmc_error{"NoMode", "Mode not available for this step"};
    }
    return nullptr;
  }

  const auto& mode = std::get<osrm::json::String>(mode_iter->second).value;
  return mode.c_str();
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

const char*
osrmc_route_response_step_driving_side(osrmc_route_response_t response,
                                       unsigned route_index,
                                       unsigned leg_index,
                                       unsigned step_index,
                                       osrmc_error_t* error) try {
  if (!osrmc_validate_route_response(response, error)) {
    return nullptr;
  }
  auto* response_holder = osrmc_get_route_response(response);
  const auto* step = osrmc_get_route_step(response_holder->json, route_index, leg_index, step_index, error);
  if (!step) {
    return nullptr;
  }

  const auto driving_side_iter = step->values.find("driving_side");
  if (driving_side_iter == step->values.end()) {
    if (error) {
      *error = new osrmc_error{"NoDrivingSide", "Driving side not available for this step"};
    }
    return nullptr;
  }

  const auto& driving_side = std::get<osrm::json::String>(driving_side_iter->second).value;
  return driving_side.c_str();
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

const char*
osrmc_route_response_step_maneuver_type(osrmc_route_response_t response,
                                        unsigned route_index,
                                        unsigned leg_index,
                                        unsigned step_index,
                                        osrmc_error_t* error) try {
  if (!osrmc_validate_route_response(response, error)) {
    return nullptr;
  }
  auto* response_holder = osrmc_get_route_response(response);
  const auto* step = osrmc_get_route_step(response_holder->json, route_index, leg_index, step_index, error);
  if (!step) {
    return nullptr;
  }

  const auto maneuver_iter = step->values.find("maneuver");
  if (maneuver_iter == step->values.end()) {
    if (error) {
      *error = new osrmc_error{"NoManeuver", "Maneuver not available for this step"};
    }
    return nullptr;
  }

  const auto& maneuver = std::get<osrm::json::Object>(maneuver_iter->second);
  const auto type_iter = maneuver.values.find("type");
  if (type_iter == maneuver.values.end()) {
    if (error) {
      *error = new osrmc_error{"NoType", "Type not available for this maneuver"};
    }
    return nullptr;
  }

  const auto& type = std::get<osrm::json::String>(type_iter->second).value;
  return type.c_str();
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

const char*
osrmc_route_response_step_maneuver_modifier(osrmc_route_response_t response,
                                            unsigned route_index,
                                            unsigned leg_index,
                                            unsigned step_index,
                                            osrmc_error_t* error) try {
  if (!osrmc_validate_route_response(response, error)) {
    return nullptr;
  }
  auto* response_holder = osrmc_get_route_response(response);
  const auto* step = osrmc_get_route_step(response_holder->json, route_index, leg_index, step_index, error);
  if (!step) {
    return nullptr;
  }

  const auto maneuver_iter = step->values.find("maneuver");
  if (maneuver_iter == step->values.end()) {
    if (error) {
      *error = new osrmc_error{"NoManeuver", "Maneuver not available for this step"};
    }
    return nullptr;
  }

  const auto& maneuver = std::get<osrm::json::Object>(maneuver_iter->second);
  const auto modifier_iter = maneuver.values.find("modifier");
  if (modifier_iter == maneuver.values.end()) {
    return nullptr;
  }

  const auto& modifier = std::get<osrm::json::String>(modifier_iter->second).value;
  return modifier.c_str();
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

double
osrmc_route_response_step_maneuver_location_latitude(osrmc_route_response_t response,
                                                     unsigned route_index,
                                                     unsigned leg_index,
                                                     unsigned step_index,
                                                     osrmc_error_t* error) try {
  if (!osrmc_validate_route_response(response, error)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  auto* response_holder = osrmc_get_route_response(response);
  const auto* step = osrmc_get_route_step(response_holder->json, route_index, leg_index, step_index, error);
  if (!step) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto maneuver_iter = step->values.find("maneuver");
  if (maneuver_iter == step->values.end()) {
    if (error) {
      *error = new osrmc_error{"NoManeuver", "Maneuver not available for this step"};
    }
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& maneuver = std::get<osrm::json::Object>(maneuver_iter->second);
  const auto location_iter = maneuver.values.find("location");
  if (location_iter == maneuver.values.end()) {
    if (error) {
      *error = new osrmc_error{"NoLocation", "Location not available for this maneuver"};
    }
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& location = std::get<osrm::json::Array>(location_iter->second);
  if (location.values.size() < 2) {
    if (error) {
      *error = new osrmc_error{"InvalidLocation", "Location array does not have enough elements"};
    }
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto latitude = std::get<osrm::json::Number>(location.values[COORDINATE_LATITUDE_INDEX]).value;
  return latitude;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::quiet_NaN();
}

double
osrmc_route_response_step_maneuver_location_longitude(osrmc_route_response_t response,
                                                      unsigned route_index,
                                                      unsigned leg_index,
                                                      unsigned step_index,
                                                      osrmc_error_t* error) try {
  if (!osrmc_validate_route_response(response, error)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  auto* response_holder = osrmc_get_route_response(response);
  const auto* step = osrmc_get_route_step(response_holder->json, route_index, leg_index, step_index, error);
  if (!step) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto maneuver_iter = step->values.find("maneuver");
  if (maneuver_iter == step->values.end()) {
    if (error) {
      *error = new osrmc_error{"NoManeuver", "Maneuver not available for this step"};
    }
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& maneuver = std::get<osrm::json::Object>(maneuver_iter->second);
  const auto location_iter = maneuver.values.find("location");
  if (location_iter == maneuver.values.end()) {
    if (error) {
      *error = new osrmc_error{"NoLocation", "Location not available for this maneuver"};
    }
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& location = std::get<osrm::json::Array>(location_iter->second);
  if (location.values.size() < 2) {
    if (error) {
      *error = new osrmc_error{"InvalidLocation", "Location array does not have enough elements"};
    }
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto longitude = std::get<osrm::json::Number>(location.values[COORDINATE_LONGITUDE_INDEX]).value;
  return longitude;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::quiet_NaN();
}

double
osrmc_route_response_step_maneuver_bearing_before(osrmc_route_response_t response,
                                                  unsigned route_index,
                                                  unsigned leg_index,
                                                  unsigned step_index,
                                                  osrmc_error_t* error) try {
  if (!osrmc_validate_route_response(response, error)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  auto* response_holder = osrmc_get_route_response(response);
  const auto* step = osrmc_get_route_step(response_holder->json, route_index, leg_index, step_index, error);
  if (!step) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto maneuver_iter = step->values.find("maneuver");
  if (maneuver_iter == step->values.end()) {
    if (error) {
      *error = new osrmc_error{"NoManeuver", "Maneuver not available for this step"};
    }
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& maneuver = std::get<osrm::json::Object>(maneuver_iter->second);
  const auto bearing_before_iter = maneuver.values.find("bearing_before");
  if (bearing_before_iter == maneuver.values.end()) {
    if (error) {
      *error = new osrmc_error{"NoBearingBefore", "Bearing before not available for this maneuver"};
    }
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto bearing_before = std::get<osrm::json::Number>(bearing_before_iter->second).value;
  return bearing_before;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::quiet_NaN();
}

double
osrmc_route_response_step_maneuver_bearing_after(osrmc_route_response_t response,
                                                 unsigned route_index,
                                                 unsigned leg_index,
                                                 unsigned step_index,
                                                 osrmc_error_t* error) try {
  if (!osrmc_validate_route_response(response, error)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  auto* response_holder = osrmc_get_route_response(response);
  const auto* step = osrmc_get_route_step(response_holder->json, route_index, leg_index, step_index, error);
  if (!step) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto maneuver_iter = step->values.find("maneuver");
  if (maneuver_iter == step->values.end()) {
    if (error) {
      *error = new osrmc_error{"NoManeuver", "Maneuver not available for this step"};
    }
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& maneuver = std::get<osrm::json::Object>(maneuver_iter->second);
  const auto bearing_after_iter = maneuver.values.find("bearing_after");
  if (bearing_after_iter == maneuver.values.end()) {
    if (error) {
      *error = new osrmc_error{"NoBearingAfter", "Bearing after not available for this maneuver"};
    }
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto bearing_after = std::get<osrm::json::Number>(bearing_after_iter->second).value;
  return bearing_after;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::quiet_NaN();
}

int
osrmc_route_response_step_maneuver_exit(osrmc_route_response_t response,
                                        unsigned route_index,
                                        unsigned leg_index,
                                        unsigned step_index,
                                        osrmc_error_t* error) try {
  if (!osrmc_validate_route_response(response, error)) {
    return -1;
  }
  auto* response_holder = osrmc_get_route_response(response);
  const auto* step = osrmc_get_route_step(response_holder->json, route_index, leg_index, step_index, error);
  if (!step) {
    return -1;
  }

  const auto maneuver_iter = step->values.find("maneuver");
  if (maneuver_iter == step->values.end()) {
    if (error) {
      *error = new osrmc_error{"NoManeuver", "Maneuver not available for this step"};
    }
    return -1;
  }

  const auto& maneuver = std::get<osrm::json::Object>(maneuver_iter->second);
  const auto exit_iter = maneuver.values.find("exit");
  if (exit_iter == maneuver.values.end()) {
    return -1;
  }

  const auto exit = std::get<osrm::json::Number>(exit_iter->second).value;
  return static_cast<int>(exit);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return -1;
}

const char*
osrmc_route_response_step_geometry_polyline(osrmc_route_response_t response,
                                            unsigned route_index,
                                            unsigned leg_index,
                                            unsigned step_index,
                                            osrmc_error_t* error) try {
  if (!osrmc_validate_route_response(response, error)) {
    return nullptr;
  }
  auto* response_holder = osrmc_get_route_response(response);
  const auto* step = osrmc_get_route_step(response_holder->json, route_index, leg_index, step_index, error);
  if (!step) {
    return nullptr;
  }

  const auto geometry_iter = step->values.find("geometry");
  if (geometry_iter == step->values.end()) {
    if (error) {
      *error = new osrmc_error{"NoGeometry", "Geometry not available for this step"};
    }
    return nullptr;
  }

  const auto& geometry = geometry_iter->second;

  // Check if it's a polyline string
  if (std::holds_alternative<osrm::json::String>(geometry)) {
    const auto& polyline = std::get<osrm::json::String>(geometry).value;
    return polyline.c_str();
  }

  // If it's GeoJSON, we can't return polyline
  if (error) {
    *error = new osrmc_error{"UnsupportedGeometry", "Step geometry is GeoJSON, not polyline"};
  }
  return nullptr;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

unsigned
osrmc_route_response_step_geometry_coordinate_count(osrmc_route_response_t response,
                                                    unsigned route_index,
                                                    unsigned leg_index,
                                                    unsigned step_index,
                                                    osrmc_error_t* error) try {
  if (!osrmc_validate_route_response(response, error)) {
    return 0;
  }
  auto* response_holder = osrmc_get_route_response(response);
  const auto* step = osrmc_get_route_step(response_holder->json, route_index, leg_index, step_index, error);
  if (!step) {
    return 0;
  }

  const auto* coordinates = osrmc_get_step_geometry_coordinates(*step, error);
  if (!coordinates) {
    return 0;
  }

  return static_cast<unsigned>(coordinates->values.size());
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return 0;
}

double
osrmc_route_response_step_geometry_coordinate_latitude(osrmc_route_response_t response,
                                                       unsigned route_index,
                                                       unsigned leg_index,
                                                       unsigned step_index,
                                                       unsigned coord_index,
                                                       osrmc_error_t* error) try {
  if (!osrmc_validate_route_response(response, error)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  auto* response_holder = osrmc_get_route_response(response);
  const auto* step = osrmc_get_route_step(response_holder->json, route_index, leg_index, step_index, error);
  if (!step) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto* coordinates = osrmc_get_step_geometry_coordinates(*step, error);
  if (!coordinates) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  if (coord_index >= coordinates->values.size()) {
    if (error) {
      *error = new osrmc_error{"IndexOutOfBounds", "Coordinate index out of bounds"};
    }
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& coord = std::get<osrm::json::Array>(coordinates->values.at(coord_index));
  if (coord.values.size() < 2) {
    if (error) {
      *error = new osrmc_error{"InvalidCoordinate", "Coordinate array does not have enough elements"};
    }
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto latitude = std::get<osrm::json::Number>(coord.values[COORDINATE_LATITUDE_INDEX]).value;
  return latitude;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::quiet_NaN();
}

double
osrmc_route_response_step_geometry_coordinate_longitude(osrmc_route_response_t response,
                                                        unsigned route_index,
                                                        unsigned leg_index,
                                                        unsigned step_index,
                                                        unsigned coord_index,
                                                        osrmc_error_t* error) try {
  if (!osrmc_validate_route_response(response, error)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  auto* response_holder = osrmc_get_route_response(response);
  const auto* step = osrmc_get_route_step(response_holder->json, route_index, leg_index, step_index, error);
  if (!step) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto* coordinates = osrmc_get_step_geometry_coordinates(*step, error);
  if (!coordinates) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  if (coord_index >= coordinates->values.size()) {
    if (error) {
      *error = new osrmc_error{"IndexOutOfBounds", "Coordinate index out of bounds"};
    }
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& coord = std::get<osrm::json::Array>(coordinates->values.at(coord_index));
  if (coord.values.size() < 2) {
    if (error) {
      *error = new osrmc_error{"InvalidCoordinate", "Coordinate array does not have enough elements"};
    }
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto longitude = std::get<osrm::json::Number>(coord.values[COORDINATE_LONGITUDE_INDEX]).value;
  return longitude;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::quiet_NaN();
}

unsigned
osrmc_route_response_step_intersection_count(osrmc_route_response_t response,
                                             unsigned route_index,
                                             unsigned leg_index,
                                             unsigned step_index,
                                             osrmc_error_t* error) try {
  if (!osrmc_validate_route_response(response, error)) {
    return 0;
  }
  auto* response_holder = osrmc_get_route_response(response);
  const auto* step = osrmc_get_route_step(response_holder->json, route_index, leg_index, step_index, error);
  if (!step) {
    return 0;
  }

  const auto intersections_iter = step->values.find("intersections");
  if (intersections_iter == step->values.end()) {
    return 0;
  }

  const auto& intersections = std::get<osrm::json::Array>(intersections_iter->second);
  return static_cast<unsigned>(intersections.values.size());
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return 0;
}

double
osrmc_route_response_step_intersection_location_latitude(osrmc_route_response_t response,
                                                         unsigned route_index,
                                                         unsigned leg_index,
                                                         unsigned step_index,
                                                         unsigned intersection_index,
                                                         osrmc_error_t* error) try {
  if (!osrmc_validate_route_response(response, error)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  auto* response_holder = osrmc_get_route_response(response);
  const auto* step = osrmc_get_route_step(response_holder->json, route_index, leg_index, step_index, error);
  if (!step) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto* intersection = osrmc_get_step_intersection(*step, intersection_index, error);
  if (!intersection) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto location_iter = intersection->values.find("location");
  if (location_iter == intersection->values.end()) {
    if (error) {
      *error = new osrmc_error{"NoLocation", "Location not available for this intersection"};
    }
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& location = std::get<osrm::json::Array>(location_iter->second);
  if (location.values.size() < 2) {
    if (error) {
      *error = new osrmc_error{"InvalidLocation", "Location array does not have enough elements"};
    }
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto latitude = std::get<osrm::json::Number>(location.values[COORDINATE_LATITUDE_INDEX]).value;
  return latitude;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::quiet_NaN();
}

double
osrmc_route_response_step_intersection_location_longitude(osrmc_route_response_t response,
                                                          unsigned route_index,
                                                          unsigned leg_index,
                                                          unsigned step_index,
                                                          unsigned intersection_index,
                                                          osrmc_error_t* error) try {
  if (!osrmc_validate_route_response(response, error)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  auto* response_holder = osrmc_get_route_response(response);
  const auto* step = osrmc_get_route_step(response_holder->json, route_index, leg_index, step_index, error);
  if (!step) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto* intersection = osrmc_get_step_intersection(*step, intersection_index, error);
  if (!intersection) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto location_iter = intersection->values.find("location");
  if (location_iter == intersection->values.end()) {
    if (error) {
      *error = new osrmc_error{"NoLocation", "Location not available for this intersection"};
    }
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& location = std::get<osrm::json::Array>(location_iter->second);
  if (location.values.size() < 2) {
    if (error) {
      *error = new osrmc_error{"InvalidLocation", "Location array does not have enough elements"};
    }
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto longitude = std::get<osrm::json::Number>(location.values[COORDINATE_LONGITUDE_INDEX]).value;
  return longitude;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::quiet_NaN();
}

unsigned
osrmc_route_response_step_intersection_bearings_count(osrmc_route_response_t response,
                                                      unsigned route_index,
                                                      unsigned leg_index,
                                                      unsigned step_index,
                                                      unsigned intersection_index,
                                                      osrmc_error_t* error) try {
  if (!osrmc_validate_route_response(response, error)) {
    return 0;
  }
  auto* response_holder = osrmc_get_route_response(response);
  const auto* step = osrmc_get_route_step(response_holder->json, route_index, leg_index, step_index, error);
  if (!step) {
    return 0;
  }

  const auto* intersection = osrmc_get_step_intersection(*step, intersection_index, error);
  if (!intersection) {
    return 0;
  }

  const auto bearings_iter = intersection->values.find("bearings");
  if (bearings_iter == intersection->values.end()) {
    return 0;
  }

  const auto& bearings = std::get<osrm::json::Array>(bearings_iter->second);
  return static_cast<unsigned>(bearings.values.size());
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return 0;
}

double
osrmc_route_response_step_intersection_bearing(osrmc_route_response_t response,
                                               unsigned route_index,
                                               unsigned leg_index,
                                               unsigned step_index,
                                               unsigned intersection_index,
                                               unsigned bearing_index,
                                               osrmc_error_t* error) try {
  if (!osrmc_validate_route_response(response, error)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  auto* response_holder = osrmc_get_route_response(response);
  const auto* step = osrmc_get_route_step(response_holder->json, route_index, leg_index, step_index, error);
  if (!step) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto* intersection = osrmc_get_step_intersection(*step, intersection_index, error);
  if (!intersection) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto bearings_iter = intersection->values.find("bearings");
  if (bearings_iter == intersection->values.end()) {
    if (error) {
      *error = new osrmc_error{"NoBearings", "Bearings not available for this intersection"};
    }
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& bearings = std::get<osrm::json::Array>(bearings_iter->second);
  if (bearing_index >= bearings.values.size()) {
    if (error) {
      *error = new osrmc_error{"IndexOutOfBounds", "Bearing index out of bounds"};
    }
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto bearing = std::get<osrm::json::Number>(bearings.values.at(bearing_index)).value;
  return bearing;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::quiet_NaN();
}

int
osrmc_route_response_step_intersection_entry(osrmc_route_response_t response,
                                             unsigned route_index,
                                             unsigned leg_index,
                                             unsigned step_index,
                                             unsigned intersection_index,
                                             unsigned entry_index,
                                             osrmc_error_t* error) try {
  if (!osrmc_validate_route_response(response, error)) {
    return -1;
  }
  auto* response_holder = osrmc_get_route_response(response);
  const auto* step = osrmc_get_route_step(response_holder->json, route_index, leg_index, step_index, error);
  if (!step) {
    return -1;
  }

  const auto* intersection = osrmc_get_step_intersection(*step, intersection_index, error);
  if (!intersection) {
    return -1;
  }

  const auto entry_iter = intersection->values.find("entry");
  if (entry_iter == intersection->values.end()) {
    if (error) {
      *error = new osrmc_error{"NoEntry", "Entry flags not available for this intersection"};
    }
    return -1;
  }

  const auto& entry = std::get<osrm::json::Array>(entry_iter->second);
  if (entry_index >= entry.values.size()) {
    if (error) {
      *error = new osrmc_error{"IndexOutOfBounds", "Entry index out of bounds"};
    }
    return -1;
  }

  const auto& entry_value = entry.values.at(entry_index);
  if (std::holds_alternative<osrm::json::True>(entry_value)) {
    return 1;
  } else if (std::holds_alternative<osrm::json::False>(entry_value)) {
    return 0;
  } else {
    if (error) {
      *error = new osrmc_error{"InvalidEntry", "Entry value is not a boolean"};
    }
    return -1;
  }
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return -1;
}

double
osrmc_route_response_step_intersection_in_bearing(osrmc_route_response_t response,
                                                  unsigned route_index,
                                                  unsigned leg_index,
                                                  unsigned step_index,
                                                  unsigned intersection_index,
                                                  osrmc_error_t* error) try {
  if (!osrmc_validate_route_response(response, error)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  auto* response_holder = osrmc_get_route_response(response);
  const auto* step = osrmc_get_route_step(response_holder->json, route_index, leg_index, step_index, error);
  if (!step) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto* intersection = osrmc_get_step_intersection(*step, intersection_index, error);
  if (!intersection) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto in_bearing_iter = intersection->values.find("in");
  if (in_bearing_iter == intersection->values.end()) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto in_bearing = std::get<osrm::json::Number>(in_bearing_iter->second).value;
  return in_bearing;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::quiet_NaN();
}

double
osrmc_route_response_step_intersection_out_bearing(osrmc_route_response_t response,
                                                   unsigned route_index,
                                                   unsigned leg_index,
                                                   unsigned step_index,
                                                   unsigned intersection_index,
                                                   osrmc_error_t* error) try {
  if (!osrmc_validate_route_response(response, error)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  auto* response_holder = osrmc_get_route_response(response);
  const auto* step = osrmc_get_route_step(response_holder->json, route_index, leg_index, step_index, error);
  if (!step) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto* intersection = osrmc_get_step_intersection(*step, intersection_index, error);
  if (!intersection) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto out_bearing_iter = intersection->values.find("out");
  if (out_bearing_iter == intersection->values.end()) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto out_bearing = std::get<osrm::json::Number>(out_bearing_iter->second).value;
  return out_bearing;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::quiet_NaN();
}

unsigned
osrmc_route_response_step_intersection_lanes_count(osrmc_route_response_t response,
                                                   unsigned route_index,
                                                   unsigned leg_index,
                                                   unsigned step_index,
                                                   unsigned intersection_index,
                                                   osrmc_error_t* error) try {
  if (!osrmc_validate_route_response(response, error)) {
    return 0;
  }
  auto* response_holder = osrmc_get_route_response(response);
  const auto* step = osrmc_get_route_step(response_holder->json, route_index, leg_index, step_index, error);
  if (!step) {
    return 0;
  }

  const auto* intersection = osrmc_get_step_intersection(*step, intersection_index, error);
  if (!intersection) {
    return 0;
  }

  const auto lanes_iter = intersection->values.find("lanes");
  if (lanes_iter == intersection->values.end()) {
    return 0;
  }

  const auto& lanes = std::get<osrm::json::Array>(lanes_iter->second);
  return static_cast<unsigned>(lanes.values.size());
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return 0;
}

unsigned
osrmc_route_response_step_intersection_lane_indications_count(osrmc_route_response_t response,
                                                              unsigned route_index,
                                                              unsigned leg_index,
                                                              unsigned step_index,
                                                              unsigned intersection_index,
                                                              unsigned lane_index,
                                                              osrmc_error_t* error) try {
  if (!osrmc_validate_route_response(response, error)) {
    return 0;
  }
  auto* response_holder = osrmc_get_route_response(response);
  const auto* step = osrmc_get_route_step(response_holder->json, route_index, leg_index, step_index, error);
  if (!step) {
    return 0;
  }

  const auto* intersection = osrmc_get_step_intersection(*step, intersection_index, error);
  if (!intersection) {
    return 0;
  }

  const auto lanes_iter = intersection->values.find("lanes");
  if (lanes_iter == intersection->values.end()) {
    if (error) {
      *error = new osrmc_error{"NoLanes", "Lanes not available for this intersection"};
    }
    return 0;
  }

  const auto& lanes = std::get<osrm::json::Array>(lanes_iter->second);
  if (lane_index >= lanes.values.size()) {
    if (error) {
      *error = new osrmc_error{"IndexOutOfBounds", "Lane index out of bounds"};
    }
    return 0;
  }

  const auto& lane = std::get<osrm::json::Object>(lanes.values.at(lane_index));
  const auto indications_iter = lane.values.find("indications");
  if (indications_iter == lane.values.end()) {
    return 0;
  }

  const auto& indications = std::get<osrm::json::Array>(indications_iter->second);
  return static_cast<unsigned>(indications.values.size());
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return 0;
}

int
osrmc_route_response_step_intersection_lane_valid(osrmc_route_response_t response,
                                                  unsigned route_index,
                                                  unsigned leg_index,
                                                  unsigned step_index,
                                                  unsigned intersection_index,
                                                  unsigned lane_index,
                                                  osrmc_error_t* error) try {
  if (!osrmc_validate_route_response(response, error)) {
    return -1;
  }
  auto* response_holder = osrmc_get_route_response(response);
  const auto* step = osrmc_get_route_step(response_holder->json, route_index, leg_index, step_index, error);
  if (!step) {
    return -1;
  }

  const auto* intersection = osrmc_get_step_intersection(*step, intersection_index, error);
  if (!intersection) {
    return -1;
  }

  const auto lanes_iter = intersection->values.find("lanes");
  if (lanes_iter == intersection->values.end()) {
    if (error) {
      *error = new osrmc_error{"NoLanes", "Lanes not available for this intersection"};
    }
    return -1;
  }

  const auto& lanes = std::get<osrm::json::Array>(lanes_iter->second);
  if (lane_index >= lanes.values.size()) {
    if (error) {
      *error = new osrmc_error{"IndexOutOfBounds", "Lane index out of bounds"};
    }
    return -1;
  }

  const auto& lane = std::get<osrm::json::Object>(lanes.values.at(lane_index));
  const auto valid_iter = lane.values.find("valid");
  if (valid_iter == lane.values.end()) {
    return -1;
  }

  const auto& valid = valid_iter->second;
  if (std::holds_alternative<osrm::json::True>(valid)) {
    return 1;
  } else if (std::holds_alternative<osrm::json::False>(valid)) {
    return 0;
  } else {
    if (error) {
      *error = new osrmc_error{"InvalidValid", "Valid value is not a boolean"};
    }
    return -1;
  }
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return -1;
}

unsigned
osrmc_route_response_step_intersection_classes_count(osrmc_route_response_t response,
                                                     unsigned route_index,
                                                     unsigned leg_index,
                                                     unsigned step_index,
                                                     unsigned intersection_index,
                                                     osrmc_error_t* error) try {
  if (!osrmc_validate_route_response(response, error)) {
    return 0;
  }
  auto* response_holder = osrmc_get_route_response(response);
  const auto* step = osrmc_get_route_step(response_holder->json, route_index, leg_index, step_index, error);
  if (!step) {
    return 0;
  }

  const auto* intersection = osrmc_get_step_intersection(*step, intersection_index, error);
  if (!intersection) {
    return 0;
  }

  const auto classes_iter = intersection->values.find("classes");
  if (classes_iter == intersection->values.end()) {
    return 0;
  }

  const auto& classes = std::get<osrm::json::Array>(classes_iter->second);
  return static_cast<unsigned>(classes.values.size());
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return 0;
}

const char*
osrmc_route_response_step_intersection_class(osrmc_route_response_t response,
                                             unsigned route_index,
                                             unsigned leg_index,
                                             unsigned step_index,
                                             unsigned intersection_index,
                                             unsigned class_index,
                                             osrmc_error_t* error) try {
  if (!osrmc_validate_route_response(response, error)) {
    return nullptr;
  }
  auto* response_holder = osrmc_get_route_response(response);
  const auto* step = osrmc_get_route_step(response_holder->json, route_index, leg_index, step_index, error);
  if (!step) {
    return nullptr;
  }

  const auto* intersection = osrmc_get_step_intersection(*step, intersection_index, error);
  if (!intersection) {
    return nullptr;
  }

  const auto classes_iter = intersection->values.find("classes");
  if (classes_iter == intersection->values.end()) {
    if (error) {
      *error = new osrmc_error{"NoClasses", "Classes not available for this intersection"};
    }
    return nullptr;
  }

  const auto& classes = std::get<osrm::json::Array>(classes_iter->second);
  if (class_index >= classes.values.size()) {
    if (error) {
      *error = new osrmc_error{"IndexOutOfBounds", "Class index out of bounds"};
    }
    return nullptr;
  }

  const auto& class_str = std::get<osrm::json::String>(classes.values.at(class_index)).value;
  return class_str.c_str();
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

const char*
osrmc_route_response_data_version(osrmc_route_response_t response, osrmc_error_t* error) try {
  if (!osrmc_validate_route_response(response, error)) {
    return nullptr;
  }
  auto* response_holder = osrmc_get_route_response(response);
  auto& response_json = response_holder->json;

  const auto data_version_iter = response_json.values.find("data_version");
  if (data_version_iter == response_json.values.end()) {
    return nullptr;
  }

  const auto& data_version = std::get<osrm::json::String>(data_version_iter->second).value;
  return data_version.c_str();
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

osrmc_blob_t
osrmc_route_response_json(osrmc_route_response_t response, osrmc_error_t* error) try {
  if (!osrmc_validate_route_response(response, error)) {
    return nullptr;
  }
  auto* response_holder = osrmc_get_route_response(response);
  return osrmc_render_json(response_holder->json);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

/* Table */

osrmc_table_params_t
osrmc_table_params_construct(osrmc_error_t* error) try {
  auto* out = new osrm::TableParameters;
  return reinterpret_cast<osrmc_table_params_t>(out);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

void
osrmc_table_params_destruct(osrmc_table_params_t params) {
  if (params) {
    delete reinterpret_cast<osrm::TableParameters*>(params);
  }
}

void
osrmc_table_params_add_source(osrmc_table_params_t params, size_t index, osrmc_error_t* error) try {
  if (!osrmc_validate_params(reinterpret_cast<osrmc_params_t>(params), error)) {
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::TableParameters*>(params);
  params_typed->sources.emplace_back(index);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_table_params_add_destination(osrmc_table_params_t params, size_t index, osrmc_error_t* error) try {
  if (!osrmc_validate_params(reinterpret_cast<osrmc_params_t>(params), error)) {
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::TableParameters*>(params);
  params_typed->destinations.emplace_back(index);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_table_params_set_annotations(osrmc_table_params_t params,
                                   osrmc_table_annotations_type_t annotations,
                                   osrmc_error_t* error) try {
  if (!osrmc_validate_params(reinterpret_cast<osrmc_params_t>(params), error)) {
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::TableParameters*>(params);
  switch (annotations) {
    case OSRMC_TABLE_ANNOTATIONS_NONE:
      params_typed->annotations = osrm::TableParameters::AnnotationsType::None;
      break;
    case OSRMC_TABLE_ANNOTATIONS_DURATION:
      params_typed->annotations = osrm::TableParameters::AnnotationsType::Duration;
      break;
    case OSRMC_TABLE_ANNOTATIONS_DISTANCE:
      params_typed->annotations = osrm::TableParameters::AnnotationsType::Distance;
      break;
    case OSRMC_TABLE_ANNOTATIONS_ALL:
      params_typed->annotations = osrm::TableParameters::AnnotationsType::All;
      break;
    default:
      if (error) {
        *error = new osrmc_error{"InvalidArgument", "Unknown annotations type"};
      }
      return;
  }
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_table_params_set_fallback_speed(osrmc_table_params_t params, double speed, osrmc_error_t* error) try {
  if (!osrmc_validate_params(reinterpret_cast<osrmc_params_t>(params), error)) {
    return;
  }
  if (speed <= 0) {
    if (error) {
      *error = new osrmc_error{"InvalidArgument", "Fallback speed must be positive"};
    }
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::TableParameters*>(params);
  params_typed->fallback_speed = speed;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_table_params_set_fallback_coordinate_type(osrmc_table_params_t params,
                                                osrmc_table_fallback_coordinate_type_t coord_type,
                                                osrmc_error_t* error) try {
  if (!osrmc_validate_params(reinterpret_cast<osrmc_params_t>(params), error)) {
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::TableParameters*>(params);
  switch (coord_type) {
    case OSRMC_TABLE_FALLBACK_COORDINATE_INPUT:
      params_typed->fallback_coordinate_type = osrm::TableParameters::FallbackCoordinateType::Input;
      break;
    case OSRMC_TABLE_FALLBACK_COORDINATE_SNAPPED:
      params_typed->fallback_coordinate_type = osrm::TableParameters::FallbackCoordinateType::Snapped;
      break;
    default:
      if (error) {
        *error = new osrmc_error{"InvalidArgument", "Unknown fallback coordinate type"};
      }
      return;
  }
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_table_params_set_scale_factor(osrmc_table_params_t params, double scale_factor, osrmc_error_t* error) try {
  if (!osrmc_validate_params(reinterpret_cast<osrmc_params_t>(params), error)) {
    return;
  }
  if (scale_factor <= 0) {
    if (error) {
      *error = new osrmc_error{"InvalidArgument", "Scale factor must be positive"};
    }
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::TableParameters*>(params);
  params_typed->scale_factor = scale_factor;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

osrmc_table_response_t
osrmc_table(osrmc_osrm_t osrm, osrmc_table_params_t params, osrmc_error_t* error) try {
  if (!osrmc_validate_osrm(osrm, error) || !params) {
    if (error) {
      *error = new osrmc_error{"InvalidArgument", "OSRM instance and params must not be null"};
    }
    return nullptr;
  }
  auto* osrm_typed = reinterpret_cast<osrm::OSRM*>(osrm);
  auto* params_typed = reinterpret_cast<osrm::TableParameters*>(params);

  osrm::engine::api::ResultT result = osrm::json::Object();
  const auto status = osrm_typed->Table(*params_typed, result);

  if (status == osrm::Status::Ok) {
    if (std::holds_alternative<osrm::json::Object>(result)) {
      auto* out = new osrm::json::Object(std::move(std::get<osrm::json::Object>(result)));
      return reinterpret_cast<osrmc_table_response_t>(out);
    } else {
      if (error) {
        *error = new osrmc_error{"InvalidResponse", "Unexpected response type"};
      }
      return nullptr;
    }
  }

  if (std::holds_alternative<osrm::json::Object>(result)) {
    osrmc_error_from_json(std::get<osrm::json::Object>(result), error);
  } else {
    if (error) {
      *error = new osrmc_error{"TableError", "Table request failed"};
    }
  }
  return nullptr;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

void
osrmc_table_response_destruct(osrmc_table_response_t response) {
  if (response) {
    delete reinterpret_cast<osrm::json::Object*>(response);
  }
}

double
osrmc_table_response_duration(osrmc_table_response_t response,
                              unsigned long from,
                              unsigned long to,
                              osrmc_error_t* error) try {
  if (!osrmc_validate_table_response(response, error)) {
    return std::numeric_limits<double>::infinity();
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  if (response_typed->values.find("durations") == response_typed->values.end()) {
    *error = new osrmc_error{"NoTable", "Table request not configured to return durations"};
    return std::numeric_limits<double>::infinity();
  }

  auto& durations = std::get<osrm::json::Array>(response_typed->values["durations"]);
  auto& durations_from_to_all = std::get<osrm::json::Array>(durations.values.at(from));
  auto nullable = durations_from_to_all.values.at(to);

  if (std::holds_alternative<osrm::json::Null>(nullable)) {
    *error = new osrmc_error{"NoRoute", "Impossible route between points"};
    return std::numeric_limits<double>::infinity();
  }
  auto duration = std::get<osrm::json::Number>(nullable).value;

  return duration;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::infinity();
}

double
osrmc_table_response_distance(osrmc_table_response_t response,
                              unsigned long from,
                              unsigned long to,
                              osrmc_error_t* error) try {
  if (!osrmc_validate_table_response(response, error)) {
    return std::numeric_limits<double>::infinity();
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  if (response_typed->values.find("distances") == response_typed->values.end()) {
    *error = new osrmc_error{"NoTable", "Table request not configured to return distances"};
    return std::numeric_limits<double>::infinity();
  }

  auto& distances = std::get<osrm::json::Array>(response_typed->values.at("distances"));
  auto& distances_from_to_all = std::get<osrm::json::Array>(distances.values.at(from));
  auto nullable = distances_from_to_all.values.at(to);

  if (std::holds_alternative<osrm::json::Null>(nullable)) {
    *error = new osrmc_error{"NoRoute", "Impossible route between points"};
    return std::numeric_limits<double>::infinity();
  }
  auto distance = std::get<osrm::json::Number>(nullable).value;

  return distance;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::infinity();
}

unsigned
osrmc_table_response_source_count(osrmc_table_response_t response, osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  if (response_typed->values.find("sources") == response_typed->values.end()) {
    if (response_typed->values.find("durations") != response_typed->values.end()) {
      const auto& durations = std::get<osrm::json::Array>(response_typed->values.at("durations"));
      return static_cast<unsigned>(durations.values.size());
    }
    return 0;
  }

  const auto& sources = std::get<osrm::json::Array>(response_typed->values.at("sources"));
  return static_cast<unsigned>(sources.values.size());
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return 0;
}

unsigned
osrmc_table_response_destination_count(osrmc_table_response_t response, osrmc_error_t* error) try {
  if (!osrmc_validate_table_response(response, error)) {
    return 0;
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  if (response_typed->values.find("destinations") == response_typed->values.end()) {
    if (response_typed->values.find("durations") != response_typed->values.end()) {
      const auto& durations = std::get<osrm::json::Array>(response_typed->values.at("durations"));
      if (!durations.values.empty()) {
        const auto& first_row = std::get<osrm::json::Array>(durations.values.at(0));
        return static_cast<unsigned>(first_row.values.size());
      }
    }
    return 0;
  }

  const auto& destinations = std::get<osrm::json::Array>(response_typed->values.at("destinations"));
  return static_cast<unsigned>(destinations.values.size());
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return 0;
}

int
osrmc_table_response_get_duration_matrix(osrmc_table_response_t response,
                                         double* matrix,
                                         size_t max_size,
                                         osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  if (response_typed->values.find("durations") == response_typed->values.end()) {
    *error = new osrmc_error{"NoTable", "Table request not configured to return durations"};
    return -1;
  }

  if (matrix == nullptr) {
    *error = new osrmc_error{"InvalidArgument", "Matrix buffer cannot be null"};
    return -1;
  }

  const auto& durations = std::get<osrm::json::Array>(response_typed->values.at("durations"));
  const size_t num_sources = durations.values.size();

  if (num_sources == 0) {
    return 0;
  }

  const auto& first_row = std::get<osrm::json::Array>(durations.values.at(0));
  const size_t num_destinations = first_row.values.size();
  const size_t required_size = num_sources * num_destinations;

  if (max_size < required_size) {
    *error = new osrmc_error{"BufferTooSmall", "Matrix buffer too small"};
    return -1;
  }

  size_t idx = 0;
  for (size_t i = 0; i < num_sources; ++i) {
    const auto& row = std::get<osrm::json::Array>(durations.values.at(i));
    for (size_t j = 0; j < num_destinations; ++j) {
      const auto& value = row.values.at(j);
      if (std::holds_alternative<osrm::json::Null>(value)) {
        matrix[idx] = std::numeric_limits<double>::infinity();
      } else {
        matrix[idx] = std::get<osrm::json::Number>(value).value;
      }
      ++idx;
    }
  }

  return static_cast<int>(required_size);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return -1;
}

int
osrmc_table_response_get_distance_matrix(osrmc_table_response_t response,
                                         double* matrix,
                                         size_t max_size,
                                         osrmc_error_t* error) try {
  if (!osrmc_validate_table_response(response, error)) {
    return -1;
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  if (response_typed->values.find("distances") == response_typed->values.end()) {
    *error = new osrmc_error{"NoTable", "Table request not configured to return distances"};
    return -1;
  }

  if (matrix == nullptr) {
    *error = new osrmc_error{"InvalidArgument", "Matrix buffer cannot be null"};
    return -1;
  }

  const auto& distances = std::get<osrm::json::Array>(response_typed->values.at("distances"));
  const size_t num_sources = distances.values.size();

  if (num_sources == 0) {
    return 0;
  }

  const auto& first_row = std::get<osrm::json::Array>(distances.values.at(0));
  const size_t num_destinations = first_row.values.size();
  const size_t required_size = num_sources * num_destinations;

  if (max_size < required_size) {
    *error = new osrmc_error{"BufferTooSmall", "Matrix buffer too small"};
    return -1;
  }

  size_t idx = 0;
  for (size_t i = 0; i < num_sources; ++i) {
    const auto& row = std::get<osrm::json::Array>(distances.values.at(i));
    for (size_t j = 0; j < num_destinations; ++j) {
      const auto& value = row.values.at(j);
      if (std::holds_alternative<osrm::json::Null>(value)) {
        matrix[idx] = std::numeric_limits<double>::infinity();
      } else {
        matrix[idx] = std::get<osrm::json::Number>(value).value;
      }
      ++idx;
    }
  }

  return static_cast<int>(required_size);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return -1;
}

double
osrmc_table_response_source_latitude(osrmc_table_response_t response, unsigned index, osrmc_error_t* error) try {
  if (!osrmc_validate_table_response(response, error)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  auto* response_json = reinterpret_cast<osrm::json::Object*>(response);

  const auto sources_iter = response_json->values.find("sources");
  if (sources_iter == response_json->values.end()) {
    *error = new osrmc_error{"NoSources", "Sources not available in response"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& sources = std::get<osrm::json::Array>(sources_iter->second);
  if (index >= sources.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Source index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& source = std::get<osrm::json::Object>(sources.values.at(index));
  const auto& location = std::get<osrm::json::Array>(source.values.at("location")).values;
  const auto latitude = std::get<osrm::json::Number>(location[COORDINATE_LATITUDE_INDEX]).value;
  return latitude;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::quiet_NaN();
}

double
osrmc_table_response_source_longitude(osrmc_table_response_t response, unsigned index, osrmc_error_t* error) try {
  if (!osrmc_validate_table_response(response, error)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  auto* response_json = reinterpret_cast<osrm::json::Object*>(response);

  const auto sources_iter = response_json->values.find("sources");
  if (sources_iter == response_json->values.end()) {
    *error = new osrmc_error{"NoSources", "Sources not available in response"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& sources = std::get<osrm::json::Array>(sources_iter->second);
  if (index >= sources.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Source index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& source = std::get<osrm::json::Object>(sources.values.at(index));
  const auto& location = std::get<osrm::json::Array>(source.values.at("location")).values;
  const auto longitude = std::get<osrm::json::Number>(location[COORDINATE_LONGITUDE_INDEX]).value;
  return longitude;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::quiet_NaN();
}

const char*
osrmc_table_response_source_name(osrmc_table_response_t response, unsigned index, osrmc_error_t* error) try {
  if (!osrmc_validate_table_response(response, error)) {
    return nullptr;
  }
  auto* response_json = reinterpret_cast<osrm::json::Object*>(response);

  const auto sources_iter = response_json->values.find("sources");
  if (sources_iter == response_json->values.end()) {
    *error = new osrmc_error{"NoSources", "Sources not available in response"};
    return nullptr;
  }

  const auto& sources = std::get<osrm::json::Array>(sources_iter->second);
  if (index >= sources.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Source index out of bounds"};
    return nullptr;
  }

  const auto& source = std::get<osrm::json::Object>(sources.values.at(index));
  const auto name_iter = source.values.find("name");
  if (name_iter == source.values.end()) {
    return "";
  }

  const auto& name = std::get<osrm::json::String>(name_iter->second).value;
  return name.c_str();
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

double
osrmc_table_response_destination_latitude(osrmc_table_response_t response, unsigned index, osrmc_error_t* error) try {
  auto* response_json = reinterpret_cast<osrm::json::Object*>(response);

  const auto destinations_iter = response_json->values.find("destinations");
  if (destinations_iter == response_json->values.end()) {
    *error = new osrmc_error{"NoDestinations", "Destinations not available in response"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& destinations = std::get<osrm::json::Array>(destinations_iter->second);
  if (index >= destinations.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Destination index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& destination = std::get<osrm::json::Object>(destinations.values.at(index));
  const auto& location = std::get<osrm::json::Array>(destination.values.at("location")).values;
  const auto latitude = std::get<osrm::json::Number>(location[COORDINATE_LATITUDE_INDEX]).value;
  return latitude;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::quiet_NaN();
}

double
osrmc_table_response_destination_longitude(osrmc_table_response_t response, unsigned index, osrmc_error_t* error) try {
  if (!osrmc_validate_table_response(response, error)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  auto* response_json = reinterpret_cast<osrm::json::Object*>(response);

  const auto destinations_iter = response_json->values.find("destinations");
  if (destinations_iter == response_json->values.end()) {
    *error = new osrmc_error{"NoDestinations", "Destinations not available in response"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& destinations = std::get<osrm::json::Array>(destinations_iter->second);
  if (index >= destinations.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Destination index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& destination = std::get<osrm::json::Object>(destinations.values.at(index));
  const auto& location = std::get<osrm::json::Array>(destination.values.at("location")).values;
  const auto longitude = std::get<osrm::json::Number>(location[COORDINATE_LONGITUDE_INDEX]).value;
  return longitude;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::quiet_NaN();
}

const char*
osrmc_table_response_destination_name(osrmc_table_response_t response, unsigned index, osrmc_error_t* error) try {
  if (!osrmc_validate_table_response(response, error)) {
    return nullptr;
  }
  auto* response_json = reinterpret_cast<osrm::json::Object*>(response);

  const auto destinations_iter = response_json->values.find("destinations");
  if (destinations_iter == response_json->values.end()) {
    *error = new osrmc_error{"NoDestinations", "Destinations not available in response"};
    return nullptr;
  }

  const auto& destinations = std::get<osrm::json::Array>(destinations_iter->second);
  if (index >= destinations.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Destination index out of bounds"};
    return nullptr;
  }

  const auto& destination = std::get<osrm::json::Object>(destinations.values.at(index));
  const auto name_iter = destination.values.find("name");
  if (name_iter == destination.values.end()) {
    return "";
  }

  const auto& name = std::get<osrm::json::String>(name_iter->second).value;
  return name.c_str();
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

unsigned
osrmc_table_response_fallback_speed_cells_count(osrmc_table_response_t response, osrmc_error_t* error) try {
  if (!osrmc_validate_table_response(response, error)) {
    return 0;
  }
  auto* response_json = reinterpret_cast<osrm::json::Object*>(response);

  const auto fallback_speed_cells_iter = response_json->values.find("fallback_speed_cells");
  if (fallback_speed_cells_iter == response_json->values.end()) {
    return 0;
  }

  const auto& fallback_speed_cells = std::get<osrm::json::Array>(fallback_speed_cells_iter->second);
  return static_cast<unsigned>(fallback_speed_cells.values.size());
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return 0;
}

unsigned
osrmc_table_response_fallback_speed_cell_row(osrmc_table_response_t response,
                                             unsigned index,
                                             osrmc_error_t* error) try {
  auto* response_json = reinterpret_cast<osrm::json::Object*>(response);

  const auto fallback_speed_cells_iter = response_json->values.find("fallback_speed_cells");
  if (fallback_speed_cells_iter == response_json->values.end()) {
    *error = new osrmc_error{"NoFallbackSpeedCells", "Fallback speed cells not available"};
    return 0;
  }

  const auto& fallback_speed_cells = std::get<osrm::json::Array>(fallback_speed_cells_iter->second);
  if (index >= fallback_speed_cells.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Fallback speed cell index out of bounds"};
    return 0;
  }

  const auto& cell = std::get<osrm::json::Array>(fallback_speed_cells.values.at(index));
  if (cell.values.size() < 1) {
    *error = new osrmc_error{"InvalidCell", "Cell array is empty"};
    return 0;
  }

  const auto row = std::get<osrm::json::Number>(cell.values[0]).value;
  return static_cast<unsigned>(row);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return 0;
}

unsigned
osrmc_table_response_fallback_speed_cell_column(osrmc_table_response_t response,
                                                unsigned index,
                                                osrmc_error_t* error) try {
  if (!osrmc_validate_table_response(response, error)) {
    return 0;
  }
  auto* response_json = reinterpret_cast<osrm::json::Object*>(response);

  const auto fallback_speed_cells_iter = response_json->values.find("fallback_speed_cells");
  if (fallback_speed_cells_iter == response_json->values.end()) {
    *error = new osrmc_error{"NoFallbackSpeedCells", "Fallback speed cells not available"};
    return 0;
  }

  const auto& fallback_speed_cells = std::get<osrm::json::Array>(fallback_speed_cells_iter->second);
  if (index >= fallback_speed_cells.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Fallback speed cell index out of bounds"};
    return 0;
  }

  const auto& cell = std::get<osrm::json::Array>(fallback_speed_cells.values.at(index));
  if (cell.values.size() < 2) {
    *error = new osrmc_error{"InvalidCell", "Cell array does not have enough elements"};
    return 0;
  }

  const auto column = std::get<osrm::json::Number>(cell.values[1]).value;
  return static_cast<unsigned>(column);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return 0;
}

const char*
osrmc_table_response_data_version(osrmc_table_response_t response, osrmc_error_t* error) try {
  if (!osrmc_validate_table_response(response, error)) {
    return nullptr;
  }
  auto* response_json = reinterpret_cast<osrm::json::Object*>(response);

  const auto data_version_iter = response_json->values.find("data_version");
  if (data_version_iter == response_json->values.end()) {
    return nullptr;
  }

  const auto& data_version = std::get<osrm::json::String>(data_version_iter->second).value;
  return data_version.c_str();
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

osrmc_blob_t
osrmc_table_response_json(osrmc_table_response_t response, osrmc_error_t* error) try {
  if (!osrmc_validate_table_response(response, error)) {
    return nullptr;
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);
  return osrmc_render_json(*response_typed);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

/* Match */

osrmc_match_params_t
osrmc_match_params_construct(osrmc_error_t* error) try {
  auto* out = new osrm::MatchParameters;
  return reinterpret_cast<osrmc_match_params_t>(out);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

void
osrmc_match_params_destruct(osrmc_match_params_t params) {
  if (params) {
    delete reinterpret_cast<osrm::MatchParameters*>(params);
  }
}

void
osrmc_match_params_set_steps(osrmc_match_params_t params, int on, osrmc_error_t* error) try {
  if (!osrmc_validate_params(reinterpret_cast<osrmc_params_t>(params), error)) {
    return;
  }
  osrmc_route_like_set_steps_flag(params, on);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_match_params_set_alternatives(osrmc_match_params_t params, int on, osrmc_error_t* error) try {
  if (!osrmc_validate_params(reinterpret_cast<osrmc_params_t>(params), error)) {
    return;
  }
  osrmc_route_like_set_alternatives_flag(params, on);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_match_params_set_geometries(osrmc_match_params_t params,
                                  osrmc_geometries_type_t geometries,
                                  osrmc_error_t* error) try {
  auto* params_typed = osrmc_route_like_params(params);
  switch (geometries) {
    case OSRMC_GEOMETRIES_POLYLINE:
      params_typed->geometries = osrm::RouteParameters::GeometriesType::Polyline;
      break;
    case OSRMC_GEOMETRIES_POLYLINE6:
      params_typed->geometries = osrm::RouteParameters::GeometriesType::Polyline6;
      break;
    case OSRMC_GEOMETRIES_GEOJSON:
      params_typed->geometries = osrm::RouteParameters::GeometriesType::GeoJSON;
      break;
    default:
      if (error) {
        *error = new osrmc_error{"InvalidArgument", "Unknown geometries type"};
      }
      return;
  }
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_match_params_set_overview(osrmc_match_params_t params, osrmc_overview_type_t overview, osrmc_error_t* error) try {
  auto* params_typed = osrmc_route_like_params(params);
  switch (overview) {
    case OSRMC_OVERVIEW_SIMPLIFIED:
      params_typed->overview = osrm::RouteParameters::OverviewType::Simplified;
      break;
    case OSRMC_OVERVIEW_FULL:
      params_typed->overview = osrm::RouteParameters::OverviewType::Full;
      break;
    case OSRMC_OVERVIEW_FALSE:
      params_typed->overview = osrm::RouteParameters::OverviewType::False;
      break;
    default:
      if (error) {
        *error = new osrmc_error{"InvalidArgument", "Unknown overview type"};
      }
      return;
  }
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_match_params_set_continue_straight(osrmc_match_params_t params, int on, osrmc_error_t* error) try {
  osrmc_route_like_set_continue_straight_impl(osrmc_route_like_params(params), on);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_match_params_set_number_of_alternatives(osrmc_match_params_t params, unsigned count, osrmc_error_t* error) try {
  osrmc_route_like_set_number_of_alternatives_value(params, count);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_match_params_set_annotations(osrmc_match_params_t params,
                                   osrmc_annotations_type_t annotations,
                                   osrmc_error_t* error) try {
  auto* params_typed = osrmc_route_like_params(params);
  params_typed->annotations_type = static_cast<osrm::RouteParameters::AnnotationsType>(annotations);
  params_typed->annotations = (annotations != OSRMC_ANNOTATIONS_NONE);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_match_params_add_waypoint(osrmc_match_params_t params, size_t index, osrmc_error_t* error) try {
  osrmc_route_like_add_waypoint_impl(osrmc_route_like_params(params), index);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_match_params_clear_waypoints(osrmc_match_params_t params) {
  osrmc_route_like_clear_waypoints_impl(osrmc_route_like_params(params));
}

void
osrmc_match_params_add_timestamp(osrmc_match_params_t params, unsigned timestamp, osrmc_error_t* error) try {
  if (!osrmc_validate_params(reinterpret_cast<osrmc_params_t>(params), error)) {
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::MatchParameters*>(params);
  params_typed->timestamps.emplace_back(timestamp);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_match_params_set_gaps(osrmc_match_params_t params, osrmc_match_gaps_type_t gaps, osrmc_error_t* error) try {
  if (!osrmc_validate_params(reinterpret_cast<osrmc_params_t>(params), error)) {
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::MatchParameters*>(params);
  params_typed->gaps = static_cast<osrm::MatchParameters::GapsType>(gaps);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_match_params_set_tidy(osrmc_match_params_t params, int on, osrmc_error_t* error) try {
  if (!osrmc_validate_params(reinterpret_cast<osrmc_params_t>(params), error)) {
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::MatchParameters*>(params);
  params_typed->tidy = on != 0;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

osrmc_match_response_t
osrmc_match(osrmc_osrm_t osrm, osrmc_match_params_t params, osrmc_error_t* error) try {
  if (!osrmc_validate_osrm(osrm, error) || !params) {
    if (error) {
      *error = new osrmc_error{"InvalidArgument", "OSRM instance and params must not be null"};
    }
    return nullptr;
  }
  auto* osrm_typed = reinterpret_cast<osrm::OSRM*>(osrm);
  auto* params_typed = reinterpret_cast<osrm::MatchParameters*>(params);

  osrm::engine::api::ResultT result = osrm::json::Object();
  const auto status = osrm_typed->Match(*params_typed, result);

  if (status == osrm::Status::Ok) {
    if (std::holds_alternative<osrm::json::Object>(result)) {
      auto* out = new osrm::json::Object(std::move(std::get<osrm::json::Object>(result)));
      return reinterpret_cast<osrmc_match_response_t>(out);
    } else {
      if (error) {
        *error = new osrmc_error{"InvalidResponse", "Unexpected response type"};
      }
      return nullptr;
    }
  }

  if (std::holds_alternative<osrm::json::Object>(result)) {
    osrmc_error_from_json(std::get<osrm::json::Object>(result), error);
  } else {
    if (error) {
      *error = new osrmc_error{"MatchError", "Match request failed"};
    }
  }
  return nullptr;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

void
osrmc_match_response_destruct(osrmc_match_response_t response) {
  if (response) {
    delete reinterpret_cast<osrm::json::Object*>(response);
  }
}

unsigned
osrmc_match_response_route_count(osrmc_match_response_t response, osrmc_error_t* error) try {
  if (!osrmc_validate_match_response(response, error)) {
    return 0;
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  if (response_typed->values.find("matchings") == response_typed->values.end()) {
    return 0;
  }

  const auto& matchings = std::get<osrm::json::Array>(response_typed->values.at("matchings"));
  return static_cast<unsigned>(matchings.values.size());
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return 0;
}

unsigned
osrmc_match_response_tracepoint_count(osrmc_match_response_t response, osrmc_error_t* error) try {
  if (!osrmc_validate_match_response(response, error)) {
    return 0;
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  if (response_typed->values.find("tracepoints") == response_typed->values.end()) {
    return 0;
  }

  const auto& tracepoints = std::get<osrm::json::Array>(response_typed->values.at("tracepoints"));
  return static_cast<unsigned>(tracepoints.values.size());
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return 0;
}

double
osrmc_match_response_route_distance(osrmc_match_response_t response, unsigned route_index, osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  const auto& matchings = std::get<osrm::json::Array>(response_typed->values.at("matchings"));
  if (route_index >= matchings.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Route index out of bounds"};
    return std::numeric_limits<double>::infinity();
  }

  const auto& route = std::get<osrm::json::Object>(matchings.values.at(route_index));
  const auto distance = std::get<osrm::json::Number>(route.values.at("distance")).value;

  return distance;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::infinity();
}

double
osrmc_match_response_route_duration(osrmc_match_response_t response, unsigned route_index, osrmc_error_t* error) try {
  if (!osrmc_validate_match_response(response, error)) {
    return std::numeric_limits<double>::infinity();
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  const auto& matchings = std::get<osrm::json::Array>(response_typed->values.at("matchings"));
  if (route_index >= matchings.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Route index out of bounds"};
    return std::numeric_limits<double>::infinity();
  }

  const auto& route = std::get<osrm::json::Object>(matchings.values.at(route_index));
  const auto duration = std::get<osrm::json::Number>(route.values.at("duration")).value;

  return duration;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::infinity();
}

double
osrmc_match_response_route_confidence(osrmc_match_response_t response, unsigned route_index, osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  const auto& matchings = std::get<osrm::json::Array>(response_typed->values.at("matchings"));
  if (route_index >= matchings.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Route index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& route = std::get<osrm::json::Object>(matchings.values.at(route_index));
  if (route.values.find("confidence") == route.values.end()) {
    *error = new osrmc_error{"NoConfidence", "Confidence not available for this route"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto confidence = std::get<osrm::json::Number>(route.values.at("confidence")).value;

  return confidence;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::quiet_NaN();
}

double
osrmc_match_response_tracepoint_latitude(osrmc_match_response_t response, unsigned index, osrmc_error_t* error) try {
  if (!osrmc_validate_match_response(response, error)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  const auto& tracepoints = std::get<osrm::json::Array>(response_typed->values.at("tracepoints"));
  if (index >= tracepoints.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Tracepoint index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& tracepoint_value = tracepoints.values.at(index);
  if (std::holds_alternative<osrm::json::Null>(tracepoint_value)) {
    *error = new osrmc_error{"NullTracepoint", "Tracepoint was omitted (outlier)"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& tracepoint = std::get<osrm::json::Object>(tracepoint_value);
  const auto& location = std::get<osrm::json::Array>(tracepoint.values.at("location")).values;
  const auto latitude = std::get<osrm::json::Number>(location[COORDINATE_LATITUDE_INDEX]).value;

  return latitude;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::quiet_NaN();
}

double
osrmc_match_response_tracepoint_longitude(osrmc_match_response_t response, unsigned index, osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  const auto& tracepoints = std::get<osrm::json::Array>(response_typed->values.at("tracepoints"));
  if (index >= tracepoints.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Tracepoint index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& tracepoint_value = tracepoints.values.at(index);
  if (std::holds_alternative<osrm::json::Null>(tracepoint_value)) {
    *error = new osrmc_error{"NullTracepoint", "Tracepoint was omitted (outlier)"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& tracepoint = std::get<osrm::json::Object>(tracepoint_value);
  const auto& location = std::get<osrm::json::Array>(tracepoint.values.at("location")).values;
  const auto longitude = std::get<osrm::json::Number>(location[COORDINATE_LONGITUDE_INDEX]).value;

  return longitude;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::quiet_NaN();
}

int
osrmc_match_response_tracepoint_is_null(osrmc_match_response_t response, unsigned index, osrmc_error_t* error) try {
  if (!osrmc_validate_match_response(response, error)) {
    return -1;
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  const auto& tracepoints = std::get<osrm::json::Array>(response_typed->values.at("tracepoints"));
  if (index >= tracepoints.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Tracepoint index out of bounds"};
    return -1;
  }

  const auto& tracepoint_value = tracepoints.values.at(index);
  return std::holds_alternative<osrm::json::Null>(tracepoint_value) ? 1 : 0;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return -1;
}

unsigned
osrmc_match_response_tracepoint_matchings_index(osrmc_match_response_t response,
                                                unsigned index,
                                                osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  const auto& tracepoints = std::get<osrm::json::Array>(response_typed->values.at("tracepoints"));
  if (index >= tracepoints.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Tracepoint index out of bounds"};
    return std::numeric_limits<unsigned>::max();
  }

  const auto& tracepoint_value = tracepoints.values.at(index);
  if (std::holds_alternative<osrm::json::Null>(tracepoint_value)) {
    *error = new osrmc_error{"NullTracepoint", "Tracepoint was omitted (outlier)"};
    return std::numeric_limits<unsigned>::max();
  }

  const auto& tracepoint = std::get<osrm::json::Object>(tracepoint_value);
  const auto matchings_index = std::get<osrm::json::Number>(tracepoint.values.at("matchings_index")).value;

  return static_cast<unsigned>(matchings_index);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<unsigned>::max();
}

int
osrmc_match_response_tracepoint_waypoint_index(osrmc_match_response_t response,
                                               unsigned index,
                                               osrmc_error_t* error) try {
  if (!osrmc_validate_match_response(response, error)) {
    return -1;
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  const auto& tracepoints = std::get<osrm::json::Array>(response_typed->values.at("tracepoints"));
  if (index >= tracepoints.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Tracepoint index out of bounds"};
    return -1;
  }

  const auto& tracepoint_value = tracepoints.values.at(index);
  if (std::holds_alternative<osrm::json::Null>(tracepoint_value)) {
    *error = new osrmc_error{"NullTracepoint", "Tracepoint was omitted (outlier)"};
    return -1;
  }

  const auto& tracepoint = std::get<osrm::json::Object>(tracepoint_value);
  const auto waypoint_index_iter = tracepoint.values.find("waypoint_index");
  if (waypoint_index_iter == tracepoint.values.end()) {
    *error = new osrmc_error{"NoWaypointIndex", "Waypoint index not available"};
    return -1;
  }

  if (std::holds_alternative<osrm::json::Null>(waypoint_index_iter->second)) {
    return -1;
  }

  const auto waypoint_index = std::get<osrm::json::Number>(waypoint_index_iter->second).value;
  return static_cast<int>(waypoint_index);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return -1;
}

unsigned
osrmc_match_response_tracepoint_alternatives_count(osrmc_match_response_t response,
                                                   unsigned index,
                                                   osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  const auto& tracepoints = std::get<osrm::json::Array>(response_typed->values.at("tracepoints"));
  if (index >= tracepoints.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Tracepoint index out of bounds"};
    return std::numeric_limits<unsigned>::max();
  }

  const auto& tracepoint_value = tracepoints.values.at(index);
  if (std::holds_alternative<osrm::json::Null>(tracepoint_value)) {
    *error = new osrmc_error{"NullTracepoint", "Tracepoint was omitted (outlier)"};
    return std::numeric_limits<unsigned>::max();
  }

  const auto& tracepoint = std::get<osrm::json::Object>(tracepoint_value);
  const auto alternatives_count = std::get<osrm::json::Number>(tracepoint.values.at("alternatives_count")).value;

  return static_cast<unsigned>(alternatives_count);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<unsigned>::max();
}

const char*
osrmc_match_response_tracepoint_name(osrmc_match_response_t response, unsigned index, osrmc_error_t* error) try {
  if (!osrmc_validate_match_response(response, error)) {
    return nullptr;
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  const auto& tracepoints = std::get<osrm::json::Array>(response_typed->values.at("tracepoints"));
  if (index >= tracepoints.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Tracepoint index out of bounds"};
    return nullptr;
  }

  const auto& tracepoint_value = tracepoints.values.at(index);
  if (std::holds_alternative<osrm::json::Null>(tracepoint_value)) {
    *error = new osrmc_error{"NullTracepoint", "Tracepoint was omitted (outlier)"};
    return nullptr;
  }

  const auto& tracepoint = std::get<osrm::json::Object>(tracepoint_value);
  const auto name_iter = tracepoint.values.find("name");
  if (name_iter == tracepoint.values.end()) {
    return nullptr;
  }

  const auto& name = std::get<osrm::json::String>(name_iter->second).value;
  return name.c_str();
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

double
osrmc_match_response_tracepoint_distance(osrmc_match_response_t response, unsigned index, osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  const auto& tracepoints = std::get<osrm::json::Array>(response_typed->values.at("tracepoints"));
  if (index >= tracepoints.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Tracepoint index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& tracepoint_value = tracepoints.values.at(index);
  if (std::holds_alternative<osrm::json::Null>(tracepoint_value)) {
    *error = new osrmc_error{"NullTracepoint", "Tracepoint was omitted (outlier)"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& tracepoint = std::get<osrm::json::Object>(tracepoint_value);
  const auto distance = std::get<osrm::json::Number>(tracepoint.values.at("distance")).value;

  return distance;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::quiet_NaN();
}

const char*
osrmc_match_response_tracepoint_hint(osrmc_match_response_t response, unsigned index, osrmc_error_t* error) try {
  if (!osrmc_validate_match_response(response, error)) {
    return nullptr;
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  const auto& tracepoints = std::get<osrm::json::Array>(response_typed->values.at("tracepoints"));
  if (index >= tracepoints.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Tracepoint index out of bounds"};
    return nullptr;
  }

  const auto& tracepoint_value = tracepoints.values.at(index);
  if (std::holds_alternative<osrm::json::Null>(tracepoint_value)) {
    *error = new osrmc_error{"NullTracepoint", "Tracepoint was omitted (outlier)"};
    return nullptr;
  }

  const auto& tracepoint = std::get<osrm::json::Object>(tracepoint_value);
  const auto hint_iter = tracepoint.values.find("hint");
  if (hint_iter == tracepoint.values.end()) {
    return nullptr;
  }

  const auto& hint = std::get<osrm::json::String>(hint_iter->second).value;
  return hint.c_str();
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

const char*
osrmc_match_response_data_version(osrmc_match_response_t response, osrmc_error_t* error) try {
  if (!osrmc_validate_match_response(response, error)) {
    return nullptr;
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  const auto data_version_iter = response_typed->values.find("data_version");
  if (data_version_iter == response_typed->values.end()) {
    return nullptr;
  }

  const auto& data_version = std::get<osrm::json::String>(data_version_iter->second).value;
  return data_version.c_str();
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

osrmc_blob_t
osrmc_match_response_json(osrmc_match_response_t response, osrmc_error_t* error) try {
  if (!osrmc_validate_match_response(response, error)) {
    return nullptr;
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);
  return osrmc_render_json(*response_typed);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

const char*
osrmc_match_response_geometry_polyline(osrmc_match_response_t response,
                                       unsigned route_index,
                                       osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  if (response_typed->values.find("matchings") == response_typed->values.end()) {
    *error = new osrmc_error{"NoMatchings", "No matchings available in response"};
    return nullptr;
  }

  const auto& matchings = std::get<osrm::json::Array>(response_typed->values.at("matchings"));
  if (route_index >= matchings.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Route index out of bounds"};
    return nullptr;
  }

  const auto& route = std::get<osrm::json::Object>(matchings.values.at(route_index));
  const auto geometry_iter = route.values.find("geometry");
  if (geometry_iter == route.values.end()) {
    *error = new osrmc_error{"NoGeometry", "Geometry not available for this route"};
    return nullptr;
  }

  const auto& geometry = geometry_iter->second;
  if (std::holds_alternative<osrm::json::String>(geometry)) {
    const auto& polyline = std::get<osrm::json::String>(geometry).value;
    return polyline.c_str();
  }

  *error = new osrmc_error{"NoPolyline", "Geometry not available as polyline for this route"};
  return nullptr;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

unsigned
osrmc_match_response_geometry_coordinate_count(osrmc_match_response_t response,
                                               unsigned route_index,
                                               osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  if (response_typed->values.find("matchings") == response_typed->values.end()) {
    if (error) {
      *error = new osrmc_error{"NoMatchings", "No matchings available in response"};
    }
    return 0;
  }

  const auto& matchings = std::get<osrm::json::Array>(response_typed->values.at("matchings"));
  if (route_index >= matchings.values.size()) {
    if (error) {
      *error = new osrmc_error{"IndexOutOfBounds", "Route index out of bounds"};
    }
    return 0;
  }

  const auto& route = std::get<osrm::json::Object>(matchings.values.at(route_index));
  const auto* coordinates = osrmc_get_route_geometry_coordinates(route, error);
  if (!coordinates) {
    return 0;
  }
  return static_cast<unsigned>(coordinates->values.size());
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return 0;
}

double
osrmc_match_response_geometry_coordinate_latitude(osrmc_match_response_t response,
                                                  unsigned route_index,
                                                  unsigned coord_index,
                                                  osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  if (response_typed->values.find("matchings") == response_typed->values.end()) {
    *error = new osrmc_error{"NoMatchings", "No matchings available in response"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& matchings = std::get<osrm::json::Array>(response_typed->values.at("matchings"));
  if (route_index >= matchings.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Route index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& route = std::get<osrm::json::Object>(matchings.values.at(route_index));
  const auto* coordinates = osrmc_get_route_geometry_coordinates(route, error);
  if (!coordinates) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  if (coord_index >= coordinates->values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Coordinate index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& coordinate_pair = std::get<osrm::json::Array>(coordinates->values.at(coord_index)).values;
  if (coordinate_pair.size() < MIN_COORDINATE_PAIR_SIZE) {
    *error = new osrmc_error{"InvalidGeometry", "Coordinate entry is malformed"};
    return std::numeric_limits<double>::quiet_NaN();
  }
  return std::get<osrm::json::Number>(coordinate_pair[COORDINATE_LATITUDE_INDEX]).value;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::quiet_NaN();
}

double
osrmc_match_response_geometry_coordinate_longitude(osrmc_match_response_t response,
                                                   unsigned route_index,
                                                   unsigned coord_index,
                                                   osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  if (response_typed->values.find("matchings") == response_typed->values.end()) {
    *error = new osrmc_error{"NoMatchings", "No matchings available in response"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& matchings = std::get<osrm::json::Array>(response_typed->values.at("matchings"));
  if (route_index >= matchings.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Route index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& route = std::get<osrm::json::Object>(matchings.values.at(route_index));
  const auto* coordinates = osrmc_get_route_geometry_coordinates(route, error);
  if (!coordinates) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  if (coord_index >= coordinates->values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Coordinate index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& coordinate_pair = std::get<osrm::json::Array>(coordinates->values.at(coord_index)).values;
  if (coordinate_pair.size() < MIN_COORDINATE_PAIR_SIZE) {
    *error = new osrmc_error{"InvalidGeometry", "Coordinate entry is malformed"};
    return std::numeric_limits<double>::quiet_NaN();
  }
  return std::get<osrm::json::Number>(coordinate_pair[COORDINATE_LONGITUDE_INDEX]).value;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::quiet_NaN();
}

double
osrmc_match_response_route_weight(osrmc_match_response_t response, unsigned route_index, osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  if (response_typed->values.find("matchings") == response_typed->values.end()) {
    *error = new osrmc_error{"NoMatchings", "No matchings available in response"};
    return std::numeric_limits<double>::infinity();
  }

  const auto& matchings = std::get<osrm::json::Array>(response_typed->values.at("matchings"));
  if (route_index >= matchings.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Route index out of bounds"};
    return std::numeric_limits<double>::infinity();
  }

  const auto& route = std::get<osrm::json::Object>(matchings.values.at(route_index));
  const auto weight_iter = route.values.find("weight");
  if (weight_iter == route.values.end()) {
    *error = new osrmc_error{"NoWeight", "Weight not available for this route"};
    return std::numeric_limits<double>::infinity();
  }

  const auto weight = std::get<osrm::json::Number>(weight_iter->second).value;
  return weight;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::infinity();
}

const char*
osrmc_match_response_route_weight_name(osrmc_match_response_t response,
                                       unsigned route_index,
                                       osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  if (response_typed->values.find("matchings") == response_typed->values.end()) {
    *error = new osrmc_error{"NoMatchings", "No matchings available in response"};
    return nullptr;
  }

  const auto& matchings = std::get<osrm::json::Array>(response_typed->values.at("matchings"));
  if (route_index >= matchings.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Route index out of bounds"};
    return nullptr;
  }

  const auto& route = std::get<osrm::json::Object>(matchings.values.at(route_index));
  const auto weight_name_iter = route.values.find("weight_name");
  if (weight_name_iter == route.values.end()) {
    *error = new osrmc_error{"NoWeightName", "Weight name not available for this route"};
    return nullptr;
  }

  const auto& weight_name = std::get<osrm::json::String>(weight_name_iter->second).value;
  return weight_name.c_str();
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

unsigned
osrmc_match_response_alternative_count(osrmc_match_response_t response, osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  if (response_typed->values.find("matchings") == response_typed->values.end()) {
    if (error) {
      *error = new osrmc_error{"NoMatchings", "No matchings available in response"};
    }
    return 0;
  }

  const auto& matchings = std::get<osrm::json::Array>(response_typed->values.at("matchings"));
  return static_cast<unsigned>(matchings.values.size());
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return 0;
}

unsigned
osrmc_match_response_leg_count(osrmc_match_response_t response, unsigned route_index, osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  if (response_typed->values.find("matchings") == response_typed->values.end()) {
    *error = new osrmc_error{"NoMatchings", "No matchings available in response"};
    return 0;
  }

  const auto& matchings = std::get<osrm::json::Array>(response_typed->values.at("matchings"));
  if (route_index >= matchings.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Route index out of bounds"};
    return 0;
  }

  const auto& route = std::get<osrm::json::Object>(matchings.values.at(route_index));
  if (route.values.find("legs") == route.values.end()) {
    return 0;
  }

  const auto& legs = std::get<osrm::json::Array>(route.values.at("legs"));
  return static_cast<unsigned>(legs.values.size());
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return 0;
}

double
osrmc_match_response_leg_weight(osrmc_match_response_t response,
                                unsigned route_index,
                                unsigned leg_index,
                                osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  if (response_typed->values.find("matchings") == response_typed->values.end()) {
    *error = new osrmc_error{"NoMatchings", "No matchings available in response"};
    return std::numeric_limits<double>::infinity();
  }

  const auto& matchings = std::get<osrm::json::Array>(response_typed->values.at("matchings"));
  if (route_index >= matchings.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Route index out of bounds"};
    return std::numeric_limits<double>::infinity();
  }

  const auto& route = std::get<osrm::json::Object>(matchings.values.at(route_index));
  const auto& legs = std::get<osrm::json::Array>(route.values.at("legs"));
  if (leg_index >= legs.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Leg index out of bounds"};
    return std::numeric_limits<double>::infinity();
  }

  const auto& leg = std::get<osrm::json::Object>(legs.values.at(leg_index));
  const auto weight_iter = leg.values.find("weight");
  if (weight_iter == leg.values.end()) {
    *error = new osrmc_error{"NoWeight", "Weight not available for this leg"};
    return std::numeric_limits<double>::infinity();
  }

  const auto weight = std::get<osrm::json::Number>(weight_iter->second).value;
  return weight;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::infinity();
}

const char*
osrmc_match_response_leg_summary(osrmc_match_response_t response,
                                 unsigned route_index,
                                 unsigned leg_index,
                                 osrmc_error_t* error) try {
  if (!osrmc_validate_match_response(response, error)) {
    return nullptr;
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);
  const auto* leg = osrmc_get_match_leg(*response_typed, route_index, leg_index, error);
  if (!leg) {
    return nullptr;
  }

  const auto summary_iter = leg->values.find("summary");
  if (summary_iter == leg->values.end()) {
    return nullptr;
  }

  const auto& summary = std::get<osrm::json::String>(summary_iter->second).value;
  return summary.c_str();
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

unsigned
osrmc_match_response_leg_annotations_count(osrmc_match_response_t response,
                                           unsigned route_index,
                                           unsigned leg_index,
                                           osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  if (response_typed->values.find("matchings") == response_typed->values.end()) {
    *error = new osrmc_error{"NoMatchings", "No matchings available in response"};
    return 0;
  }

  const auto& matchings = std::get<osrm::json::Array>(response_typed->values.at("matchings"));
  if (route_index >= matchings.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Route index out of bounds"};
    return 0;
  }

  const auto& route = std::get<osrm::json::Object>(matchings.values.at(route_index));
  const auto& legs = std::get<osrm::json::Array>(route.values.at("legs"));
  if (leg_index >= legs.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Leg index out of bounds"};
    return 0;
  }

  const auto& leg = std::get<osrm::json::Object>(legs.values.at(leg_index));
  const auto annotations_iter = leg.values.find("annotation");
  if (annotations_iter == leg.values.end()) {
    return 0;
  }

  const auto& annotation = std::get<osrm::json::Object>(annotations_iter->second);

  if (annotation.values.find("duration") != annotation.values.end()) {
    const auto& duration_array = std::get<osrm::json::Array>(annotation.values.at("duration"));
    return static_cast<unsigned>(duration_array.values.size());
  }
  if (annotation.values.find("distance") != annotation.values.end()) {
    const auto& distance_array = std::get<osrm::json::Array>(annotation.values.at("distance"));
    return static_cast<unsigned>(distance_array.values.size());
  }
  if (annotation.values.find("speed") != annotation.values.end()) {
    const auto& speed_array = std::get<osrm::json::Array>(annotation.values.at("speed"));
    return static_cast<unsigned>(speed_array.values.size());
  }
  if (annotation.values.find("weight") != annotation.values.end()) {
    const auto& weight_array = std::get<osrm::json::Array>(annotation.values.at("weight"));
    return static_cast<unsigned>(weight_array.values.size());
  }
  if (annotation.values.find("nodes") != annotation.values.end()) {
    const auto& nodes_array = std::get<osrm::json::Array>(annotation.values.at("nodes"));
    return static_cast<unsigned>(nodes_array.values.size());
  }
  if (annotation.values.find("datasources") != annotation.values.end()) {
    const auto& datasources_array = std::get<osrm::json::Array>(annotation.values.at("datasources"));
    return static_cast<unsigned>(datasources_array.values.size());
  }

  return 0;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return 0;
}

double
osrmc_match_response_leg_annotations_speed(osrmc_match_response_t response,
                                           unsigned route_index,
                                           unsigned leg_index,
                                           unsigned annotation_index,
                                           osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  if (response_typed->values.find("matchings") == response_typed->values.end()) {
    *error = new osrmc_error{"NoMatchings", "No matchings available in response"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& matchings = std::get<osrm::json::Array>(response_typed->values.at("matchings"));
  if (route_index >= matchings.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Route index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& route = std::get<osrm::json::Object>(matchings.values.at(route_index));
  const auto& legs = std::get<osrm::json::Array>(route.values.at("legs"));
  if (leg_index >= legs.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Leg index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& leg = std::get<osrm::json::Object>(legs.values.at(leg_index));
  const auto annotations_iter = leg.values.find("annotation");
  if (annotations_iter == leg.values.end()) {
    *error = new osrmc_error{"NoAnnotations", "Annotations not available for this leg"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& annotation = std::get<osrm::json::Object>(annotations_iter->second);
  const auto speed_iter = annotation.values.find("speed");
  if (speed_iter == annotation.values.end()) {
    *error = new osrmc_error{"NoSpeed", "Speed annotations not available"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& speed_array = std::get<osrm::json::Array>(speed_iter->second);
  if (annotation_index >= speed_array.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Annotation index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto speed = std::get<osrm::json::Number>(speed_array.values.at(annotation_index)).value;
  return speed;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::quiet_NaN();
}

double
osrmc_match_response_leg_annotations_duration(osrmc_match_response_t response,
                                              unsigned route_index,
                                              unsigned leg_index,
                                              unsigned annotation_index,
                                              osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  if (response_typed->values.find("matchings") == response_typed->values.end()) {
    *error = new osrmc_error{"NoMatchings", "No matchings available in response"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& matchings = std::get<osrm::json::Array>(response_typed->values.at("matchings"));
  if (route_index >= matchings.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Route index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& route = std::get<osrm::json::Object>(matchings.values.at(route_index));
  const auto& legs = std::get<osrm::json::Array>(route.values.at("legs"));
  if (leg_index >= legs.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Leg index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& leg = std::get<osrm::json::Object>(legs.values.at(leg_index));
  const auto annotations_iter = leg.values.find("annotation");
  if (annotations_iter == leg.values.end()) {
    *error = new osrmc_error{"NoAnnotations", "Annotations not available for this leg"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& annotation = std::get<osrm::json::Object>(annotations_iter->second);
  const auto duration_iter = annotation.values.find("duration");
  if (duration_iter == annotation.values.end()) {
    *error = new osrmc_error{"NoDuration", "Duration annotations not available"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& duration_array = std::get<osrm::json::Array>(duration_iter->second);
  if (annotation_index >= duration_array.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Annotation index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto duration = std::get<osrm::json::Number>(duration_array.values.at(annotation_index)).value;
  return duration;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::quiet_NaN();
}

double
osrmc_match_response_leg_annotations_distance(osrmc_match_response_t response,
                                              unsigned route_index,
                                              unsigned leg_index,
                                              unsigned annotation_index,
                                              osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  if (response_typed->values.find("matchings") == response_typed->values.end()) {
    *error = new osrmc_error{"NoMatchings", "No matchings available in response"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& matchings = std::get<osrm::json::Array>(response_typed->values.at("matchings"));
  if (route_index >= matchings.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Route index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& route = std::get<osrm::json::Object>(matchings.values.at(route_index));
  const auto& legs = std::get<osrm::json::Array>(route.values.at("legs"));
  if (leg_index >= legs.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Leg index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& leg = std::get<osrm::json::Object>(legs.values.at(leg_index));
  const auto annotations_iter = leg.values.find("annotation");
  if (annotations_iter == leg.values.end()) {
    *error = new osrmc_error{"NoAnnotations", "Annotations not available for this leg"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& annotation = std::get<osrm::json::Object>(annotations_iter->second);
  const auto distance_iter = annotation.values.find("distance");
  if (distance_iter == annotation.values.end()) {
    *error = new osrmc_error{"NoDistance", "Distance annotations not available"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& distance_array = std::get<osrm::json::Array>(distance_iter->second);
  if (annotation_index >= distance_array.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Annotation index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto distance = std::get<osrm::json::Number>(distance_array.values.at(annotation_index)).value;
  return distance;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::quiet_NaN();
}

double
osrmc_match_response_leg_annotations_weight(osrmc_match_response_t response,
                                            unsigned route_index,
                                            unsigned leg_index,
                                            unsigned annotation_index,
                                            osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  if (response_typed->values.find("matchings") == response_typed->values.end()) {
    *error = new osrmc_error{"NoMatchings", "No matchings available in response"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& matchings = std::get<osrm::json::Array>(response_typed->values.at("matchings"));
  if (route_index >= matchings.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Route index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& route = std::get<osrm::json::Object>(matchings.values.at(route_index));
  const auto& legs = std::get<osrm::json::Array>(route.values.at("legs"));
  if (leg_index >= legs.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Leg index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& leg = std::get<osrm::json::Object>(legs.values.at(leg_index));
  const auto annotations_iter = leg.values.find("annotation");
  if (annotations_iter == leg.values.end()) {
    *error = new osrmc_error{"NoAnnotations", "Annotations not available for this leg"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& annotation = std::get<osrm::json::Object>(annotations_iter->second);
  const auto weight_iter = annotation.values.find("weight");
  if (weight_iter == annotation.values.end()) {
    *error = new osrmc_error{"NoWeight", "Weight annotations not available"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& weight_array = std::get<osrm::json::Array>(weight_iter->second);
  if (annotation_index >= weight_array.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Annotation index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto weight = std::get<osrm::json::Number>(weight_array.values.at(annotation_index)).value;
  return weight;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::quiet_NaN();
}

unsigned
osrmc_match_response_leg_annotations_datasources(osrmc_match_response_t response,
                                                 unsigned route_index,
                                                 unsigned leg_index,
                                                 unsigned annotation_index,
                                                 osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  if (response_typed->values.find("matchings") == response_typed->values.end()) {
    *error = new osrmc_error{"NoMatchings", "No matchings available in response"};
    return 0;
  }

  const auto& matchings = std::get<osrm::json::Array>(response_typed->values.at("matchings"));
  if (route_index >= matchings.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Route index out of bounds"};
    return 0;
  }

  const auto& route = std::get<osrm::json::Object>(matchings.values.at(route_index));
  const auto& legs = std::get<osrm::json::Array>(route.values.at("legs"));
  if (leg_index >= legs.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Leg index out of bounds"};
    return 0;
  }

  const auto& leg = std::get<osrm::json::Object>(legs.values.at(leg_index));
  const auto annotations_iter = leg.values.find("annotation");
  if (annotations_iter == leg.values.end()) {
    *error = new osrmc_error{"NoAnnotations", "Annotations not available for this leg"};
    return 0;
  }

  const auto& annotation = std::get<osrm::json::Object>(annotations_iter->second);
  const auto datasources_iter = annotation.values.find("datasources");
  if (datasources_iter == annotation.values.end()) {
    *error = new osrmc_error{"NoDatasources", "Datasources annotations not available"};
    return 0;
  }

  const auto& datasources_array = std::get<osrm::json::Array>(datasources_iter->second);
  if (annotation_index >= datasources_array.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Annotation index out of bounds"};
    return 0;
  }

  const auto datasource = std::get<osrm::json::Number>(datasources_array.values.at(annotation_index)).value;
  return static_cast<unsigned>(datasource);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return 0;
}

unsigned long long
osrmc_match_response_leg_annotations_nodes(osrmc_match_response_t response,
                                           unsigned route_index,
                                           unsigned leg_index,
                                           unsigned annotation_index,
                                           osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  if (response_typed->values.find("matchings") == response_typed->values.end()) {
    *error = new osrmc_error{"NoMatchings", "No matchings available in response"};
    return 0;
  }

  const auto& matchings = std::get<osrm::json::Array>(response_typed->values.at("matchings"));
  if (route_index >= matchings.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Route index out of bounds"};
    return 0;
  }

  const auto& route = std::get<osrm::json::Object>(matchings.values.at(route_index));
  const auto& legs = std::get<osrm::json::Array>(route.values.at("legs"));
  if (leg_index >= legs.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Leg index out of bounds"};
    return 0;
  }

  const auto& leg = std::get<osrm::json::Object>(legs.values.at(leg_index));
  const auto annotations_iter = leg.values.find("annotation");
  if (annotations_iter == leg.values.end()) {
    *error = new osrmc_error{"NoAnnotations", "Annotations not available for this leg"};
    return 0;
  }

  const auto& annotation = std::get<osrm::json::Object>(annotations_iter->second);
  const auto nodes_iter = annotation.values.find("nodes");
  if (nodes_iter == annotation.values.end()) {
    *error = new osrmc_error{"NoNodes", "Nodes annotations not available"};
    return 0;
  }

  const auto& nodes_array = std::get<osrm::json::Array>(nodes_iter->second);
  if (annotation_index >= nodes_array.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Annotation index out of bounds"};
    return 0;
  }

  const auto node = std::get<osrm::json::Number>(nodes_array.values.at(annotation_index)).value;
  return static_cast<unsigned long long>(node);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return 0;
}

unsigned
osrmc_match_response_step_count(osrmc_match_response_t response,
                                unsigned route_index,
                                unsigned leg_index,
                                osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  if (response_typed->values.find("matchings") == response_typed->values.end()) {
    *error = new osrmc_error{"NoMatchings", "No matchings available in response"};
    return 0;
  }

  const auto& matchings = std::get<osrm::json::Array>(response_typed->values.at("matchings"));
  if (route_index >= matchings.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Route index out of bounds"};
    return 0;
  }

  const auto& route = std::get<osrm::json::Object>(matchings.values.at(route_index));
  const auto& legs = std::get<osrm::json::Array>(route.values.at("legs"));
  if (leg_index >= legs.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Leg index out of bounds"};
    return 0;
  }

  const auto& leg = std::get<osrm::json::Object>(legs.values.at(leg_index));
  if (leg.values.find("steps") == leg.values.end()) {
    return 0;
  }

  const auto& steps = std::get<osrm::json::Array>(leg.values.at("steps"));
  return static_cast<unsigned>(steps.values.size());
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return 0;
}

double
osrmc_match_response_step_distance(osrmc_match_response_t response,
                                   unsigned route_index,
                                   unsigned leg_index,
                                   unsigned step_index,
                                   osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  if (response_typed->values.find("matchings") == response_typed->values.end()) {
    *error = new osrmc_error{"NoMatchings", "No matchings available in response"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& matchings = std::get<osrm::json::Array>(response_typed->values.at("matchings"));
  if (route_index >= matchings.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Route index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& route = std::get<osrm::json::Object>(matchings.values.at(route_index));
  const auto& legs = std::get<osrm::json::Array>(route.values.at("legs"));
  if (leg_index >= legs.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Leg index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& leg = std::get<osrm::json::Object>(legs.values.at(leg_index));
  const auto& steps = std::get<osrm::json::Array>(leg.values.at("steps"));
  if (step_index >= steps.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Step index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& step = std::get<osrm::json::Object>(steps.values.at(step_index));
  const auto distance = std::get<osrm::json::Number>(step.values.at("distance")).value;

  return distance;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::quiet_NaN();
}

double
osrmc_match_response_step_duration(osrmc_match_response_t response,
                                   unsigned route_index,
                                   unsigned leg_index,
                                   unsigned step_index,
                                   osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  if (response_typed->values.find("matchings") == response_typed->values.end()) {
    *error = new osrmc_error{"NoMatchings", "No matchings available in response"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& matchings = std::get<osrm::json::Array>(response_typed->values.at("matchings"));
  if (route_index >= matchings.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Route index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& route = std::get<osrm::json::Object>(matchings.values.at(route_index));
  const auto& legs = std::get<osrm::json::Array>(route.values.at("legs"));
  if (leg_index >= legs.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Leg index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& leg = std::get<osrm::json::Object>(legs.values.at(leg_index));
  const auto& steps = std::get<osrm::json::Array>(leg.values.at("steps"));
  if (step_index >= steps.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Step index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& step = std::get<osrm::json::Object>(steps.values.at(step_index));
  const auto duration = std::get<osrm::json::Number>(step.values.at("duration")).value;

  return duration;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::quiet_NaN();
}

double
osrmc_match_response_step_weight(osrmc_match_response_t response,
                                 unsigned route_index,
                                 unsigned leg_index,
                                 unsigned step_index,
                                 osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  if (response_typed->values.find("matchings") == response_typed->values.end()) {
    *error = new osrmc_error{"NoMatchings", "No matchings available in response"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& matchings = std::get<osrm::json::Array>(response_typed->values.at("matchings"));
  if (route_index >= matchings.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Route index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& route = std::get<osrm::json::Object>(matchings.values.at(route_index));
  const auto& legs = std::get<osrm::json::Array>(route.values.at("legs"));
  if (leg_index >= legs.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Leg index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& leg = std::get<osrm::json::Object>(legs.values.at(leg_index));
  const auto& steps = std::get<osrm::json::Array>(leg.values.at("steps"));
  if (step_index >= steps.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Step index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& step = std::get<osrm::json::Object>(steps.values.at(step_index));
  const auto weight_iter = step.values.find("weight");
  if (weight_iter == step.values.end()) {
    *error = new osrmc_error{"NoWeight", "Weight not available for this step"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto weight = std::get<osrm::json::Number>(weight_iter->second).value;
  return weight;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::quiet_NaN();
}

const char*
osrmc_match_response_step_name(osrmc_match_response_t response,
                               unsigned route_index,
                               unsigned leg_index,
                               unsigned step_index,
                               osrmc_error_t* error) try {
  if (!osrmc_validate_match_response(response, error)) {
    return nullptr;
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);
  const auto* step = osrmc_get_match_step(*response_typed, route_index, leg_index, step_index, error);
  if (!step) {
    return nullptr;
  }

  const auto name_iter = step->values.find("name");
  if (name_iter == step->values.end()) {
    if (error) {
      *error = new osrmc_error{"NoName", "Name not available for this step"};
    }
    return nullptr;
  }

  const auto& name = std::get<osrm::json::String>(name_iter->second).value;
  return name.c_str();
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

const char*
osrmc_match_response_step_ref(osrmc_match_response_t response,
                              unsigned route_index,
                              unsigned leg_index,
                              unsigned step_index,
                              osrmc_error_t* error) try {
  if (!osrmc_validate_match_response(response, error)) {
    return nullptr;
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);
  const auto* step = osrmc_get_match_step(*response_typed, route_index, leg_index, step_index, error);
  if (!step) {
    return nullptr;
  }

  const auto ref_iter = step->values.find("ref");
  if (ref_iter == step->values.end()) {
    return nullptr;
  }

  const auto& ref = std::get<osrm::json::String>(ref_iter->second).value;
  return ref.c_str();
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

const char*
osrmc_match_response_step_pronunciation(osrmc_match_response_t response,
                                        unsigned route_index,
                                        unsigned leg_index,
                                        unsigned step_index,
                                        osrmc_error_t* error) try {
  if (!osrmc_validate_match_response(response, error)) {
    return nullptr;
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);
  const auto* step = osrmc_get_match_step(*response_typed, route_index, leg_index, step_index, error);
  if (!step) {
    return nullptr;
  }

  const auto pronunciation_iter = step->values.find("pronunciation");
  if (pronunciation_iter == step->values.end()) {
    return nullptr;
  }

  const auto& pronunciation = std::get<osrm::json::String>(pronunciation_iter->second).value;
  return pronunciation.c_str();
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

const char*
osrmc_match_response_step_destinations(osrmc_match_response_t response,
                                       unsigned route_index,
                                       unsigned leg_index,
                                       unsigned step_index,
                                       osrmc_error_t* error) try {
  if (!osrmc_validate_match_response(response, error)) {
    return nullptr;
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);
  const auto* step = osrmc_get_match_step(*response_typed, route_index, leg_index, step_index, error);
  if (!step) {
    return nullptr;
  }

  const auto destinations_iter = step->values.find("destinations");
  if (destinations_iter == step->values.end()) {
    return nullptr;
  }

  const auto& destinations = std::get<osrm::json::String>(destinations_iter->second).value;
  return destinations.c_str();
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

const char*
osrmc_match_response_step_exits(osrmc_match_response_t response,
                                unsigned route_index,
                                unsigned leg_index,
                                unsigned step_index,
                                osrmc_error_t* error) try {
  if (!osrmc_validate_match_response(response, error)) {
    return nullptr;
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);
  const auto* step = osrmc_get_match_step(*response_typed, route_index, leg_index, step_index, error);
  if (!step) {
    return nullptr;
  }

  const auto exits_iter = step->values.find("exits");
  if (exits_iter == step->values.end()) {
    return nullptr;
  }

  const auto& exits = std::get<osrm::json::String>(exits_iter->second).value;
  return exits.c_str();
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

const char*
osrmc_match_response_step_rotary_name(osrmc_match_response_t response,
                                      unsigned route_index,
                                      unsigned leg_index,
                                      unsigned step_index,
                                      osrmc_error_t* error) try {
  if (!osrmc_validate_match_response(response, error)) {
    return nullptr;
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);
  const auto* step = osrmc_get_match_step(*response_typed, route_index, leg_index, step_index, error);
  if (!step) {
    return nullptr;
  }

  const auto rotary_name_iter = step->values.find("rotary_name");
  if (rotary_name_iter == step->values.end()) {
    return nullptr;
  }

  const auto& rotary_name = std::get<osrm::json::String>(rotary_name_iter->second).value;
  return rotary_name.c_str();
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

const char*
osrmc_match_response_step_rotary_pronunciation(osrmc_match_response_t response,
                                               unsigned route_index,
                                               unsigned leg_index,
                                               unsigned step_index,
                                               osrmc_error_t* error) try {
  if (!osrmc_validate_match_response(response, error)) {
    return nullptr;
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);
  const auto* step = osrmc_get_match_step(*response_typed, route_index, leg_index, step_index, error);
  if (!step) {
    return nullptr;
  }

  const auto rotary_pronunciation_iter = step->values.find("rotary_pronunciation");
  if (rotary_pronunciation_iter == step->values.end()) {
    return nullptr;
  }

  const auto& rotary_pronunciation = std::get<osrm::json::String>(rotary_pronunciation_iter->second).value;
  return rotary_pronunciation.c_str();
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

const char*
osrmc_match_response_step_mode(osrmc_match_response_t response,
                               unsigned route_index,
                               unsigned leg_index,
                               unsigned step_index,
                               osrmc_error_t* error) try {
  if (!osrmc_validate_match_response(response, error)) {
    return nullptr;
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);
  const auto* step = osrmc_get_match_step(*response_typed, route_index, leg_index, step_index, error);
  if (!step) {
    return nullptr;
  }

  const auto mode_iter = step->values.find("mode");
  if (mode_iter == step->values.end()) {
    if (error) {
      *error = new osrmc_error{"NoMode", "Mode not available for this step"};
    }
    return nullptr;
  }

  const auto& mode = std::get<osrm::json::String>(mode_iter->second).value;
  return mode.c_str();
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

const char*
osrmc_match_response_step_driving_side(osrmc_match_response_t response,
                                       unsigned route_index,
                                       unsigned leg_index,
                                       unsigned step_index,
                                       osrmc_error_t* error) try {
  if (!osrmc_validate_match_response(response, error)) {
    return nullptr;
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);
  const auto* step = osrmc_get_match_step(*response_typed, route_index, leg_index, step_index, error);
  if (!step) {
    return nullptr;
  }

  const auto driving_side_iter = step->values.find("driving_side");
  if (driving_side_iter == step->values.end()) {
    if (error) {
      *error = new osrmc_error{"NoDrivingSide", "Driving side not available for this step"};
    }
    return nullptr;
  }

  const auto& driving_side = std::get<osrm::json::String>(driving_side_iter->second).value;
  return driving_side.c_str();
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

const char*
osrmc_match_response_step_maneuver_type(osrmc_match_response_t response,
                                        unsigned route_index,
                                        unsigned leg_index,
                                        unsigned step_index,
                                        osrmc_error_t* error) try {
  if (!osrmc_validate_match_response(response, error)) {
    return nullptr;
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);
  const auto* step = osrmc_get_match_step(*response_typed, route_index, leg_index, step_index, error);
  if (!step) {
    return nullptr;
  }

  const auto maneuver_iter = step->values.find("maneuver");
  if (maneuver_iter == step->values.end()) {
    if (error) {
      *error = new osrmc_error{"NoManeuver", "Maneuver not available for this step"};
    }
    return nullptr;
  }

  const auto& maneuver = std::get<osrm::json::Object>(maneuver_iter->second);
  const auto type_iter = maneuver.values.find("type");
  if (type_iter == maneuver.values.end()) {
    if (error) {
      *error = new osrmc_error{"NoType", "Type not available for this maneuver"};
    }
    return nullptr;
  }

  const auto& type = std::get<osrm::json::String>(type_iter->second).value;
  return type.c_str();
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

const char*
osrmc_match_response_step_maneuver_modifier(osrmc_match_response_t response,
                                            unsigned route_index,
                                            unsigned leg_index,
                                            unsigned step_index,
                                            osrmc_error_t* error) try {
  if (!osrmc_validate_match_response(response, error)) {
    return nullptr;
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);
  const auto* step = osrmc_get_match_step(*response_typed, route_index, leg_index, step_index, error);
  if (!step) {
    return nullptr;
  }

  const auto maneuver_iter = step->values.find("maneuver");
  if (maneuver_iter == step->values.end()) {
    if (error) {
      *error = new osrmc_error{"NoManeuver", "Maneuver not available for this step"};
    }
    return nullptr;
  }

  const auto& maneuver = std::get<osrm::json::Object>(maneuver_iter->second);
  const auto modifier_iter = maneuver.values.find("modifier");
  if (modifier_iter == maneuver.values.end()) {
    return nullptr;
  }

  const auto& modifier = std::get<osrm::json::String>(modifier_iter->second).value;
  return modifier.c_str();
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

double
osrmc_match_response_step_maneuver_location_latitude(osrmc_match_response_t response,
                                                     unsigned route_index,
                                                     unsigned leg_index,
                                                     unsigned step_index,
                                                     osrmc_error_t* error) try {
  if (!osrmc_validate_match_response(response, error)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);
  const auto* step = osrmc_get_match_step(*response_typed, route_index, leg_index, step_index, error);
  if (!step) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto maneuver_iter = step->values.find("maneuver");
  if (maneuver_iter == step->values.end()) {
    if (error) {
      *error = new osrmc_error{"NoManeuver", "Maneuver not available for this step"};
    }
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& maneuver = std::get<osrm::json::Object>(maneuver_iter->second);
  const auto location_iter = maneuver.values.find("location");
  if (location_iter == maneuver.values.end()) {
    if (error) {
      *error = new osrmc_error{"NoLocation", "Location not available for this maneuver"};
    }
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& location = std::get<osrm::json::Array>(location_iter->second);
  if (location.values.size() < 2) {
    if (error) {
      *error = new osrmc_error{"InvalidLocation", "Location array does not have enough elements"};
    }
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto latitude = std::get<osrm::json::Number>(location.values[COORDINATE_LATITUDE_INDEX]).value;
  return latitude;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::quiet_NaN();
}

double
osrmc_match_response_step_maneuver_location_longitude(osrmc_match_response_t response,
                                                      unsigned route_index,
                                                      unsigned leg_index,
                                                      unsigned step_index,
                                                      osrmc_error_t* error) try {
  if (!osrmc_validate_match_response(response, error)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);
  const auto* step = osrmc_get_match_step(*response_typed, route_index, leg_index, step_index, error);
  if (!step) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto maneuver_iter = step->values.find("maneuver");
  if (maneuver_iter == step->values.end()) {
    if (error) {
      *error = new osrmc_error{"NoManeuver", "Maneuver not available for this step"};
    }
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& maneuver = std::get<osrm::json::Object>(maneuver_iter->second);
  const auto location_iter = maneuver.values.find("location");
  if (location_iter == maneuver.values.end()) {
    if (error) {
      *error = new osrmc_error{"NoLocation", "Location not available for this maneuver"};
    }
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& location = std::get<osrm::json::Array>(location_iter->second);
  if (location.values.size() < 2) {
    if (error) {
      *error = new osrmc_error{"InvalidLocation", "Location array does not have enough elements"};
    }
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto longitude = std::get<osrm::json::Number>(location.values[COORDINATE_LONGITUDE_INDEX]).value;
  return longitude;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::quiet_NaN();
}

double
osrmc_match_response_step_maneuver_bearing_before(osrmc_match_response_t response,
                                                  unsigned route_index,
                                                  unsigned leg_index,
                                                  unsigned step_index,
                                                  osrmc_error_t* error) try {
  if (!osrmc_validate_match_response(response, error)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);
  const auto* step = osrmc_get_match_step(*response_typed, route_index, leg_index, step_index, error);
  if (!step) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto maneuver_iter = step->values.find("maneuver");
  if (maneuver_iter == step->values.end()) {
    if (error) {
      *error = new osrmc_error{"NoManeuver", "Maneuver not available for this step"};
    }
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& maneuver = std::get<osrm::json::Object>(maneuver_iter->second);
  const auto bearing_before_iter = maneuver.values.find("bearing_before");
  if (bearing_before_iter == maneuver.values.end()) {
    if (error) {
      *error = new osrmc_error{"NoBearingBefore", "Bearing before not available for this maneuver"};
    }
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto bearing_before = std::get<osrm::json::Number>(bearing_before_iter->second).value;
  return bearing_before;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::quiet_NaN();
}

double
osrmc_match_response_step_maneuver_bearing_after(osrmc_match_response_t response,
                                                 unsigned route_index,
                                                 unsigned leg_index,
                                                 unsigned step_index,
                                                 osrmc_error_t* error) try {
  if (!osrmc_validate_match_response(response, error)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);
  const auto* step = osrmc_get_match_step(*response_typed, route_index, leg_index, step_index, error);
  if (!step) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto maneuver_iter = step->values.find("maneuver");
  if (maneuver_iter == step->values.end()) {
    if (error) {
      *error = new osrmc_error{"NoManeuver", "Maneuver not available for this step"};
    }
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& maneuver = std::get<osrm::json::Object>(maneuver_iter->second);
  const auto bearing_after_iter = maneuver.values.find("bearing_after");
  if (bearing_after_iter == maneuver.values.end()) {
    if (error) {
      *error = new osrmc_error{"NoBearingAfter", "Bearing after not available for this maneuver"};
    }
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto bearing_after = std::get<osrm::json::Number>(bearing_after_iter->second).value;
  return bearing_after;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::quiet_NaN();
}

int
osrmc_match_response_step_maneuver_exit(osrmc_match_response_t response,
                                        unsigned route_index,
                                        unsigned leg_index,
                                        unsigned step_index,
                                        osrmc_error_t* error) try {
  if (!osrmc_validate_match_response(response, error)) {
    return -1;
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);
  const auto* step = osrmc_get_match_step(*response_typed, route_index, leg_index, step_index, error);
  if (!step) {
    return -1;
  }

  const auto maneuver_iter = step->values.find("maneuver");
  if (maneuver_iter == step->values.end()) {
    if (error) {
      *error = new osrmc_error{"NoManeuver", "Maneuver not available for this step"};
    }
    return -1;
  }

  const auto& maneuver = std::get<osrm::json::Object>(maneuver_iter->second);
  const auto exit_iter = maneuver.values.find("exit");
  if (exit_iter == maneuver.values.end()) {
    return -1;
  }

  const auto exit = std::get<osrm::json::Number>(exit_iter->second).value;
  return static_cast<int>(exit);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return -1;
}

const char*
osrmc_match_response_step_geometry_polyline(osrmc_match_response_t response,
                                            unsigned route_index,
                                            unsigned leg_index,
                                            unsigned step_index,
                                            osrmc_error_t* error) try {
  if (!osrmc_validate_match_response(response, error)) {
    return nullptr;
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);
  const auto* step = osrmc_get_match_step(*response_typed, route_index, leg_index, step_index, error);
  if (!step) {
    return nullptr;
  }

  const auto geometry_iter = step->values.find("geometry");
  if (geometry_iter == step->values.end()) {
    if (error) {
      *error = new osrmc_error{"NoGeometry", "Geometry not available for this step"};
    }
    return nullptr;
  }

  const auto& geometry = geometry_iter->second;

  if (std::holds_alternative<osrm::json::String>(geometry)) {
    const auto& polyline = std::get<osrm::json::String>(geometry).value;
    return polyline.c_str();
  }

  if (error) {
    *error = new osrmc_error{"UnsupportedGeometry", "Step geometry is GeoJSON, not polyline"};
  }
  return nullptr;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

unsigned
osrmc_match_response_step_geometry_coordinate_count(osrmc_match_response_t response,
                                                    unsigned route_index,
                                                    unsigned leg_index,
                                                    unsigned step_index,
                                                    osrmc_error_t* error) try {
  if (!osrmc_validate_match_response(response, error)) {
    return 0;
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);
  const auto* step = osrmc_get_match_step(*response_typed, route_index, leg_index, step_index, error);
  if (!step) {
    return 0;
  }

  const auto* coordinates = osrmc_get_step_geometry_coordinates(*step, error);
  if (!coordinates) {
    return 0;
  }

  return static_cast<unsigned>(coordinates->values.size());
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return 0;
}

double
osrmc_match_response_step_geometry_coordinate_latitude(osrmc_match_response_t response,
                                                       unsigned route_index,
                                                       unsigned leg_index,
                                                       unsigned step_index,
                                                       unsigned coord_index,
                                                       osrmc_error_t* error) try {
  if (!osrmc_validate_match_response(response, error)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);
  const auto* step = osrmc_get_match_step(*response_typed, route_index, leg_index, step_index, error);
  if (!step) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto* coordinates = osrmc_get_step_geometry_coordinates(*step, error);
  if (!coordinates) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  if (coord_index >= coordinates->values.size()) {
    if (error) {
      *error = new osrmc_error{"IndexOutOfBounds", "Coordinate index out of bounds"};
    }
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& coord = std::get<osrm::json::Array>(coordinates->values.at(coord_index));
  if (coord.values.size() < 2) {
    if (error) {
      *error = new osrmc_error{"InvalidCoordinate", "Coordinate array does not have enough elements"};
    }
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto latitude = std::get<osrm::json::Number>(coord.values[COORDINATE_LATITUDE_INDEX]).value;
  return latitude;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::quiet_NaN();
}

double
osrmc_match_response_step_geometry_coordinate_longitude(osrmc_match_response_t response,
                                                        unsigned route_index,
                                                        unsigned leg_index,
                                                        unsigned step_index,
                                                        unsigned coord_index,
                                                        osrmc_error_t* error) try {
  if (!osrmc_validate_match_response(response, error)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);
  const auto* step = osrmc_get_match_step(*response_typed, route_index, leg_index, step_index, error);
  if (!step) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto* coordinates = osrmc_get_step_geometry_coordinates(*step, error);
  if (!coordinates) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  if (coord_index >= coordinates->values.size()) {
    if (error) {
      *error = new osrmc_error{"IndexOutOfBounds", "Coordinate index out of bounds"};
    }
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& coord = std::get<osrm::json::Array>(coordinates->values.at(coord_index));
  if (coord.values.size() < 2) {
    if (error) {
      *error = new osrmc_error{"InvalidCoordinate", "Coordinate array does not have enough elements"};
    }
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto longitude = std::get<osrm::json::Number>(coord.values[COORDINATE_LONGITUDE_INDEX]).value;
  return longitude;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::quiet_NaN();
}

unsigned
osrmc_match_response_step_intersection_count(osrmc_match_response_t response,
                                             unsigned route_index,
                                             unsigned leg_index,
                                             unsigned step_index,
                                             osrmc_error_t* error) try {
  if (!osrmc_validate_match_response(response, error)) {
    return 0;
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);
  const auto* step = osrmc_get_match_step(*response_typed, route_index, leg_index, step_index, error);
  if (!step) {
    return 0;
  }

  const auto intersections_iter = step->values.find("intersections");
  if (intersections_iter == step->values.end()) {
    return 0;
  }

  const auto& intersections = std::get<osrm::json::Array>(intersections_iter->second);
  return static_cast<unsigned>(intersections.values.size());
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return 0;
}

double
osrmc_match_response_step_intersection_location_latitude(osrmc_match_response_t response,
                                                         unsigned route_index,
                                                         unsigned leg_index,
                                                         unsigned step_index,
                                                         unsigned intersection_index,
                                                         osrmc_error_t* error) try {
  if (!osrmc_validate_match_response(response, error)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);
  const auto* step = osrmc_get_match_step(*response_typed, route_index, leg_index, step_index, error);
  if (!step) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto* intersection = osrmc_get_step_intersection(*step, intersection_index, error);
  if (!intersection) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto location_iter = intersection->values.find("location");
  if (location_iter == intersection->values.end()) {
    if (error) {
      *error = new osrmc_error{"NoLocation", "Location not available for this intersection"};
    }
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& location = std::get<osrm::json::Array>(location_iter->second);
  if (location.values.size() < 2) {
    if (error) {
      *error = new osrmc_error{"InvalidLocation", "Location array does not have enough elements"};
    }
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto latitude = std::get<osrm::json::Number>(location.values[COORDINATE_LATITUDE_INDEX]).value;
  return latitude;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::quiet_NaN();
}

double
osrmc_match_response_step_intersection_location_longitude(osrmc_match_response_t response,
                                                          unsigned route_index,
                                                          unsigned leg_index,
                                                          unsigned step_index,
                                                          unsigned intersection_index,
                                                          osrmc_error_t* error) try {
  if (!osrmc_validate_match_response(response, error)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);
  const auto* step = osrmc_get_match_step(*response_typed, route_index, leg_index, step_index, error);
  if (!step) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto* intersection = osrmc_get_step_intersection(*step, intersection_index, error);
  if (!intersection) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto location_iter = intersection->values.find("location");
  if (location_iter == intersection->values.end()) {
    if (error) {
      *error = new osrmc_error{"NoLocation", "Location not available for this intersection"};
    }
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& location = std::get<osrm::json::Array>(location_iter->second);
  if (location.values.size() < 2) {
    if (error) {
      *error = new osrmc_error{"InvalidLocation", "Location array does not have enough elements"};
    }
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto longitude = std::get<osrm::json::Number>(location.values[COORDINATE_LONGITUDE_INDEX]).value;
  return longitude;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::quiet_NaN();
}

unsigned
osrmc_match_response_step_intersection_bearings_count(osrmc_match_response_t response,
                                                      unsigned route_index,
                                                      unsigned leg_index,
                                                      unsigned step_index,
                                                      unsigned intersection_index,
                                                      osrmc_error_t* error) try {
  if (!osrmc_validate_match_response(response, error)) {
    return 0;
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);
  const auto* step = osrmc_get_match_step(*response_typed, route_index, leg_index, step_index, error);
  if (!step) {
    return 0;
  }

  const auto* intersection = osrmc_get_step_intersection(*step, intersection_index, error);
  if (!intersection) {
    return 0;
  }

  const auto bearings_iter = intersection->values.find("bearings");
  if (bearings_iter == intersection->values.end()) {
    return 0;
  }

  const auto& bearings = std::get<osrm::json::Array>(bearings_iter->second);
  return static_cast<unsigned>(bearings.values.size());
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return 0;
}

double
osrmc_match_response_step_intersection_bearing(osrmc_match_response_t response,
                                               unsigned route_index,
                                               unsigned leg_index,
                                               unsigned step_index,
                                               unsigned intersection_index,
                                               unsigned bearing_index,
                                               osrmc_error_t* error) try {
  if (!osrmc_validate_match_response(response, error)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);
  const auto* step = osrmc_get_match_step(*response_typed, route_index, leg_index, step_index, error);
  if (!step) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto* intersection = osrmc_get_step_intersection(*step, intersection_index, error);
  if (!intersection) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto bearings_iter = intersection->values.find("bearings");
  if (bearings_iter == intersection->values.end()) {
    if (error) {
      *error = new osrmc_error{"NoBearings", "Bearings not available for this intersection"};
    }
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& bearings = std::get<osrm::json::Array>(bearings_iter->second);
  if (bearing_index >= bearings.values.size()) {
    if (error) {
      *error = new osrmc_error{"IndexOutOfBounds", "Bearing index out of bounds"};
    }
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto bearing = std::get<osrm::json::Number>(bearings.values.at(bearing_index)).value;
  return bearing;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::quiet_NaN();
}

int
osrmc_match_response_step_intersection_entry(osrmc_match_response_t response,
                                             unsigned route_index,
                                             unsigned leg_index,
                                             unsigned step_index,
                                             unsigned intersection_index,
                                             unsigned entry_index,
                                             osrmc_error_t* error) try {
  if (!osrmc_validate_match_response(response, error)) {
    return -1;
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);
  const auto* step = osrmc_get_match_step(*response_typed, route_index, leg_index, step_index, error);
  if (!step) {
    return -1;
  }

  const auto* intersection = osrmc_get_step_intersection(*step, intersection_index, error);
  if (!intersection) {
    return -1;
  }

  const auto entry_iter = intersection->values.find("entry");
  if (entry_iter == intersection->values.end()) {
    if (error) {
      *error = new osrmc_error{"NoEntry", "Entry flags not available for this intersection"};
    }
    return -1;
  }

  const auto& entry = std::get<osrm::json::Array>(entry_iter->second);
  if (entry_index >= entry.values.size()) {
    if (error) {
      *error = new osrmc_error{"IndexOutOfBounds", "Entry index out of bounds"};
    }
    return -1;
  }

  const auto& entry_value = entry.values.at(entry_index);
  if (std::holds_alternative<osrm::json::True>(entry_value)) {
    return 1;
  } else if (std::holds_alternative<osrm::json::False>(entry_value)) {
    return 0;
  } else {
    if (error) {
      *error = new osrmc_error{"InvalidEntry", "Entry value is not a boolean"};
    }
    return -1;
  }
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return -1;
}

double
osrmc_match_response_step_intersection_in_bearing(osrmc_match_response_t response,
                                                  unsigned route_index,
                                                  unsigned leg_index,
                                                  unsigned step_index,
                                                  unsigned intersection_index,
                                                  osrmc_error_t* error) try {
  if (!osrmc_validate_match_response(response, error)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);
  const auto* step = osrmc_get_match_step(*response_typed, route_index, leg_index, step_index, error);
  if (!step) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto* intersection = osrmc_get_step_intersection(*step, intersection_index, error);
  if (!intersection) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto in_bearing_iter = intersection->values.find("in");
  if (in_bearing_iter == intersection->values.end()) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto in_bearing = std::get<osrm::json::Number>(in_bearing_iter->second).value;
  return in_bearing;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::quiet_NaN();
}

double
osrmc_match_response_step_intersection_out_bearing(osrmc_match_response_t response,
                                                   unsigned route_index,
                                                   unsigned leg_index,
                                                   unsigned step_index,
                                                   unsigned intersection_index,
                                                   osrmc_error_t* error) try {
  if (!osrmc_validate_match_response(response, error)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);
  const auto* step = osrmc_get_match_step(*response_typed, route_index, leg_index, step_index, error);
  if (!step) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto* intersection = osrmc_get_step_intersection(*step, intersection_index, error);
  if (!intersection) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto out_bearing_iter = intersection->values.find("out");
  if (out_bearing_iter == intersection->values.end()) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto out_bearing = std::get<osrm::json::Number>(out_bearing_iter->second).value;
  return out_bearing;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::quiet_NaN();
}

unsigned
osrmc_match_response_step_intersection_lanes_count(osrmc_match_response_t response,
                                                   unsigned route_index,
                                                   unsigned leg_index,
                                                   unsigned step_index,
                                                   unsigned intersection_index,
                                                   osrmc_error_t* error) try {
  if (!osrmc_validate_match_response(response, error)) {
    return 0;
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);
  const auto* step = osrmc_get_match_step(*response_typed, route_index, leg_index, step_index, error);
  if (!step) {
    return 0;
  }

  const auto* intersection = osrmc_get_step_intersection(*step, intersection_index, error);
  if (!intersection) {
    return 0;
  }

  const auto lanes_iter = intersection->values.find("lanes");
  if (lanes_iter == intersection->values.end()) {
    return 0;
  }

  const auto& lanes = std::get<osrm::json::Array>(lanes_iter->second);
  return static_cast<unsigned>(lanes.values.size());
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return 0;
}

unsigned
osrmc_match_response_step_intersection_lane_indications_count(osrmc_match_response_t response,
                                                              unsigned route_index,
                                                              unsigned leg_index,
                                                              unsigned step_index,
                                                              unsigned intersection_index,
                                                              unsigned lane_index,
                                                              osrmc_error_t* error) try {
  if (!osrmc_validate_match_response(response, error)) {
    return 0;
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);
  const auto* step = osrmc_get_match_step(*response_typed, route_index, leg_index, step_index, error);
  if (!step) {
    return 0;
  }

  const auto* intersection = osrmc_get_step_intersection(*step, intersection_index, error);
  if (!intersection) {
    return 0;
  }

  const auto lanes_iter = intersection->values.find("lanes");
  if (lanes_iter == intersection->values.end()) {
    if (error) {
      *error = new osrmc_error{"NoLanes", "Lanes not available for this intersection"};
    }
    return 0;
  }

  const auto& lanes = std::get<osrm::json::Array>(lanes_iter->second);
  if (lane_index >= lanes.values.size()) {
    if (error) {
      *error = new osrmc_error{"IndexOutOfBounds", "Lane index out of bounds"};
    }
    return 0;
  }

  const auto& lane = std::get<osrm::json::Object>(lanes.values.at(lane_index));
  const auto indications_iter = lane.values.find("indications");
  if (indications_iter == lane.values.end()) {
    return 0;
  }

  const auto& indications = std::get<osrm::json::Array>(indications_iter->second);
  return static_cast<unsigned>(indications.values.size());
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return 0;
}

int
osrmc_match_response_step_intersection_lane_valid(osrmc_match_response_t response,
                                                  unsigned route_index,
                                                  unsigned leg_index,
                                                  unsigned step_index,
                                                  unsigned intersection_index,
                                                  unsigned lane_index,
                                                  osrmc_error_t* error) try {
  if (!osrmc_validate_match_response(response, error)) {
    return -1;
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);
  const auto* step = osrmc_get_match_step(*response_typed, route_index, leg_index, step_index, error);
  if (!step) {
    return -1;
  }

  const auto* intersection = osrmc_get_step_intersection(*step, intersection_index, error);
  if (!intersection) {
    return -1;
  }

  const auto lanes_iter = intersection->values.find("lanes");
  if (lanes_iter == intersection->values.end()) {
    if (error) {
      *error = new osrmc_error{"NoLanes", "Lanes not available for this intersection"};
    }
    return -1;
  }

  const auto& lanes = std::get<osrm::json::Array>(lanes_iter->second);
  if (lane_index >= lanes.values.size()) {
    if (error) {
      *error = new osrmc_error{"IndexOutOfBounds", "Lane index out of bounds"};
    }
    return -1;
  }

  const auto& lane = std::get<osrm::json::Object>(lanes.values.at(lane_index));
  const auto valid_iter = lane.values.find("valid");
  if (valid_iter == lane.values.end()) {
    return -1;
  }

  const auto& valid = valid_iter->second;
  if (std::holds_alternative<osrm::json::True>(valid)) {
    return 1;
  } else if (std::holds_alternative<osrm::json::False>(valid)) {
    return 0;
  } else {
    if (error) {
      *error = new osrmc_error{"InvalidValid", "Valid value is not a boolean"};
    }
    return -1;
  }
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return -1;
}

unsigned
osrmc_match_response_step_intersection_classes_count(osrmc_match_response_t response,
                                                     unsigned route_index,
                                                     unsigned leg_index,
                                                     unsigned step_index,
                                                     unsigned intersection_index,
                                                     osrmc_error_t* error) try {
  if (!osrmc_validate_match_response(response, error)) {
    return 0;
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);
  const auto* step = osrmc_get_match_step(*response_typed, route_index, leg_index, step_index, error);
  if (!step) {
    return 0;
  }

  const auto* intersection = osrmc_get_step_intersection(*step, intersection_index, error);
  if (!intersection) {
    return 0;
  }

  const auto classes_iter = intersection->values.find("classes");
  if (classes_iter == intersection->values.end()) {
    return 0;
  }

  const auto& classes = std::get<osrm::json::Array>(classes_iter->second);
  return static_cast<unsigned>(classes.values.size());
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return 0;
}

const char*
osrmc_match_response_step_intersection_class(osrmc_match_response_t response,
                                             unsigned route_index,
                                             unsigned leg_index,
                                             unsigned step_index,
                                             unsigned intersection_index,
                                             unsigned class_index,
                                             osrmc_error_t* error) try {
  if (!osrmc_validate_match_response(response, error)) {
    return nullptr;
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);
  const auto* step = osrmc_get_match_step(*response_typed, route_index, leg_index, step_index, error);
  if (!step) {
    return nullptr;
  }

  const auto* intersection = osrmc_get_step_intersection(*step, intersection_index, error);
  if (!intersection) {
    return nullptr;
  }

  const auto classes_iter = intersection->values.find("classes");
  if (classes_iter == intersection->values.end()) {
    if (error) {
      *error = new osrmc_error{"NoClasses", "Classes not available for this intersection"};
    }
    return nullptr;
  }

  const auto& classes = std::get<osrm::json::Array>(classes_iter->second);
  if (class_index >= classes.values.size()) {
    if (error) {
      *error = new osrmc_error{"IndexOutOfBounds", "Class index out of bounds"};
    }
    return nullptr;
  }

  const auto& class_str = std::get<osrm::json::String>(classes.values.at(class_index)).value;
  return class_str.c_str();
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

/* Trip */

osrmc_trip_params_t
osrmc_trip_params_construct(osrmc_error_t* error) try {
  auto* out = new osrm::TripParameters;
  return reinterpret_cast<osrmc_trip_params_t>(out);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

void
osrmc_trip_params_destruct(osrmc_trip_params_t params) {
  if (params) {
    delete reinterpret_cast<osrm::TripParameters*>(params);
  }
}

void
osrmc_trip_params_set_roundtrip(osrmc_trip_params_t params, int on, osrmc_error_t* error) try {
  if (!osrmc_validate_params(reinterpret_cast<osrmc_params_t>(params), error)) {
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::TripParameters*>(params);
  params_typed->roundtrip = (on != 0);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_trip_params_set_source(osrmc_trip_params_t params, osrmc_trip_source_type_t source, osrmc_error_t* error) try {
  if (!osrmc_validate_params(reinterpret_cast<osrmc_params_t>(params), error)) {
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::TripParameters*>(params);
  params_typed->source = static_cast<osrm::TripParameters::SourceType>(source);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_trip_params_set_destination(osrmc_trip_params_t params,
                                  osrmc_trip_destination_type_t destination,
                                  osrmc_error_t* error) try {
  if (!osrmc_validate_params(reinterpret_cast<osrmc_params_t>(params), error)) {
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::TripParameters*>(params);
  params_typed->destination = static_cast<osrm::TripParameters::DestinationType>(destination);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_trip_params_set_steps(osrmc_trip_params_t params, int on, osrmc_error_t* error) try {
  if (!osrmc_validate_params(reinterpret_cast<osrmc_params_t>(params), error)) {
    return;
  }
  osrmc_route_like_set_steps_flag(params, on);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_trip_params_set_alternatives(osrmc_trip_params_t params, int on, osrmc_error_t* error) try {
  if (!osrmc_validate_params(reinterpret_cast<osrmc_params_t>(params), error)) {
    return;
  }
  osrmc_route_like_set_alternatives_flag(params, on);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_trip_params_set_geometries(osrmc_trip_params_t params,
                                 osrmc_geometries_type_t geometries,
                                 osrmc_error_t* error) try {
  auto* params_typed = osrmc_route_like_params(params);
  switch (geometries) {
    case OSRMC_GEOMETRIES_POLYLINE:
      params_typed->geometries = osrm::RouteParameters::GeometriesType::Polyline;
      break;
    case OSRMC_GEOMETRIES_POLYLINE6:
      params_typed->geometries = osrm::RouteParameters::GeometriesType::Polyline6;
      break;
    case OSRMC_GEOMETRIES_GEOJSON:
      params_typed->geometries = osrm::RouteParameters::GeometriesType::GeoJSON;
      break;
    default:
      if (error) {
        *error = new osrmc_error{"InvalidArgument", "Unknown geometries type"};
      }
      return;
  }
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_trip_params_set_overview(osrmc_trip_params_t params, osrmc_overview_type_t overview, osrmc_error_t* error) try {
  auto* params_typed = osrmc_route_like_params(params);
  switch (overview) {
    case OSRMC_OVERVIEW_SIMPLIFIED:
      params_typed->overview = osrm::RouteParameters::OverviewType::Simplified;
      break;
    case OSRMC_OVERVIEW_FULL:
      params_typed->overview = osrm::RouteParameters::OverviewType::Full;
      break;
    case OSRMC_OVERVIEW_FALSE:
      params_typed->overview = osrm::RouteParameters::OverviewType::False;
      break;
    default:
      if (error) {
        *error = new osrmc_error{"InvalidArgument", "Unknown overview type"};
      }
      return;
  }
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_trip_params_set_continue_straight(osrmc_trip_params_t params, int on, osrmc_error_t* error) try {
  osrmc_route_like_set_continue_straight_impl(osrmc_route_like_params(params), on);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_trip_params_set_number_of_alternatives(osrmc_trip_params_t params, unsigned count, osrmc_error_t* error) try {
  osrmc_route_like_set_number_of_alternatives_value(params, count);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_trip_params_set_annotations(osrmc_trip_params_t params,
                                  osrmc_annotations_type_t annotations,
                                  osrmc_error_t* error) try {
  auto* params_typed = osrmc_route_like_params(params);
  params_typed->annotations_type = static_cast<osrm::RouteParameters::AnnotationsType>(annotations);
  params_typed->annotations = (annotations != OSRMC_ANNOTATIONS_NONE);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_trip_params_clear_waypoints(osrmc_trip_params_t params) {
  auto* params_typed = reinterpret_cast<osrm::TripParameters*>(params);
  params_typed->waypoints.clear();
}

void
osrmc_trip_params_add_waypoint(osrmc_trip_params_t params, size_t index, osrmc_error_t* error) try {
  if (!osrmc_validate_params(reinterpret_cast<osrmc_params_t>(params), error)) {
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::TripParameters*>(params);
  params_typed->waypoints.emplace_back(index);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

osrmc_trip_response_t
osrmc_trip(osrmc_osrm_t osrm, osrmc_trip_params_t params, osrmc_error_t* error) try {
  if (!osrmc_validate_osrm(osrm, error) || !params) {
    if (error) {
      *error = new osrmc_error{"InvalidArgument", "OSRM instance and params must not be null"};
    }
    return nullptr;
  }
  auto* osrm_typed = reinterpret_cast<osrm::OSRM*>(osrm);
  auto* params_typed = reinterpret_cast<osrm::TripParameters*>(params);

  osrm::engine::api::ResultT result = osrm::json::Object();
  const auto status = osrm_typed->Trip(*params_typed, result);

  if (status == osrm::Status::Ok) {
    if (std::holds_alternative<osrm::json::Object>(result)) {
      auto* out = new osrm::json::Object(std::move(std::get<osrm::json::Object>(result)));
      return reinterpret_cast<osrmc_trip_response_t>(out);
    } else {
      if (error) {
        *error = new osrmc_error{"InvalidResponse", "Unexpected response type"};
      }
      return nullptr;
    }
  }

  if (std::holds_alternative<osrm::json::Object>(result)) {
    osrmc_error_from_json(std::get<osrm::json::Object>(result), error);
  } else {
    if (error) {
      *error = new osrmc_error{"TripError", "Trip request failed"};
    }
  }
  return nullptr;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

void
osrmc_trip_response_destruct(osrmc_trip_response_t response) {
  if (response) {
    delete reinterpret_cast<osrm::json::Object*>(response);
  }
}

double
osrmc_trip_response_distance(osrmc_trip_response_t response, osrmc_error_t* error) try {
  if (!osrmc_validate_trip_response(response, error)) {
    return std::numeric_limits<double>::infinity();
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  const auto& trips = std::get<osrm::json::Array>(response_typed->values.at("trips"));
  if (trips.values.empty()) {
    *error = new osrmc_error{"NoTrips", "No trips available in response"};
    return std::numeric_limits<double>::infinity();
  }

  const auto& trip = std::get<osrm::json::Object>(trips.values.at(0));
  const auto distance = std::get<osrm::json::Number>(trip.values.at("distance")).value;
  return distance;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::infinity();
}

double
osrmc_trip_response_duration(osrmc_trip_response_t response, osrmc_error_t* error) try {
  if (!osrmc_validate_trip_response(response, error)) {
    return std::numeric_limits<double>::infinity();
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  const auto& trips = std::get<osrm::json::Array>(response_typed->values.at("trips"));
  if (trips.values.empty()) {
    *error = new osrmc_error{"NoTrips", "No trips available in response"};
    return std::numeric_limits<double>::infinity();
  }

  const auto& trip = std::get<osrm::json::Object>(trips.values.at(0));
  const auto duration = std::get<osrm::json::Number>(trip.values.at("duration")).value;
  return duration;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::infinity();
}

unsigned
osrmc_trip_response_waypoint_count(osrmc_trip_response_t response, osrmc_error_t* error) try {
  if (!osrmc_validate_trip_response(response, error)) {
    return 0;
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  if (response_typed->values.find("waypoints") == response_typed->values.end()) {
    return 0;
  }

  const auto& waypoints = std::get<osrm::json::Array>(response_typed->values.at("waypoints"));
  return static_cast<unsigned>(waypoints.values.size());
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return 0;
}

double
osrmc_trip_response_waypoint_latitude(osrmc_trip_response_t response, unsigned index, osrmc_error_t* error) try {
  if (!osrmc_validate_trip_response(response, error)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  const auto& waypoints = std::get<osrm::json::Array>(response_typed->values.at("waypoints"));
  if (index >= waypoints.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Waypoint index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& waypoint = std::get<osrm::json::Object>(waypoints.values.at(index));
  const auto& location = std::get<osrm::json::Array>(waypoint.values.at("location")).values;
  const auto latitude = std::get<osrm::json::Number>(location[COORDINATE_LATITUDE_INDEX]).value;

  return latitude;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::quiet_NaN();
}

double
osrmc_trip_response_waypoint_longitude(osrmc_trip_response_t response, unsigned index, osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  const auto& waypoints = std::get<osrm::json::Array>(response_typed->values.at("waypoints"));
  if (index >= waypoints.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Waypoint index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& waypoint = std::get<osrm::json::Object>(waypoints.values.at(index));
  const auto& location = std::get<osrm::json::Array>(waypoint.values.at("location")).values;
  const auto longitude = std::get<osrm::json::Number>(location[COORDINATE_LONGITUDE_INDEX]).value;

  return longitude;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::quiet_NaN();
}

unsigned
osrmc_trip_response_waypoint_trips_index(osrmc_trip_response_t response, unsigned index, osrmc_error_t* error) try {
  if (!osrmc_validate_trip_response(response, error)) {
    return std::numeric_limits<unsigned>::max();
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  const auto& waypoints = std::get<osrm::json::Array>(response_typed->values.at("waypoints"));
  if (index >= waypoints.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Waypoint index out of bounds"};
    return std::numeric_limits<unsigned>::max();
  }

  const auto& waypoint = std::get<osrm::json::Object>(waypoints.values.at(index));
  const auto trips_index = std::get<osrm::json::Number>(waypoint.values.at("trips_index")).value;

  return static_cast<unsigned>(trips_index);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<unsigned>::max();
}

unsigned
osrmc_trip_response_waypoint_waypoint_index(osrmc_trip_response_t response, unsigned index, osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  const auto& waypoints = std::get<osrm::json::Array>(response_typed->values.at("waypoints"));
  if (index >= waypoints.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Waypoint index out of bounds"};
    return std::numeric_limits<unsigned>::max();
  }

  const auto& waypoint = std::get<osrm::json::Object>(waypoints.values.at(index));
  const auto waypoint_index = std::get<osrm::json::Number>(waypoint.values.at("waypoint_index")).value;

  return static_cast<unsigned>(waypoint_index);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<unsigned>::max();
}

const char*
osrmc_trip_response_waypoint_name(osrmc_trip_response_t response, unsigned index, osrmc_error_t* error) try {
  if (!osrmc_validate_trip_response(response, error)) {
    return nullptr;
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  const auto& waypoints = std::get<osrm::json::Array>(response_typed->values.at("waypoints"));
  if (index >= waypoints.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Waypoint index out of bounds"};
    return nullptr;
  }

  const auto& waypoint = std::get<osrm::json::Object>(waypoints.values.at(index));
  const auto name_iter = waypoint.values.find("name");
  if (name_iter == waypoint.values.end()) {
    return nullptr;
  }

  const auto& name = std::get<osrm::json::String>(name_iter->second).value;
  return name.c_str();
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

double
osrmc_trip_response_waypoint_distance(osrmc_trip_response_t response, unsigned index, osrmc_error_t* error) try {
  if (!osrmc_validate_trip_response(response, error)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  const auto& waypoints = std::get<osrm::json::Array>(response_typed->values.at("waypoints"));
  if (index >= waypoints.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Waypoint index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& waypoint = std::get<osrm::json::Object>(waypoints.values.at(index));
  const auto distance = std::get<osrm::json::Number>(waypoint.values.at("distance")).value;

  return distance;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::quiet_NaN();
}

const char*
osrmc_trip_response_waypoint_hint(osrmc_trip_response_t response, unsigned index, osrmc_error_t* error) try {
  if (!osrmc_validate_trip_response(response, error)) {
    return nullptr;
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  const auto& waypoints = std::get<osrm::json::Array>(response_typed->values.at("waypoints"));
  if (index >= waypoints.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Waypoint index out of bounds"};
    return nullptr;
  }

  const auto& waypoint = std::get<osrm::json::Object>(waypoints.values.at(index));
  const auto hint_iter = waypoint.values.find("hint");
  if (hint_iter == waypoint.values.end()) {
    return nullptr;
  }

  const auto& hint = std::get<osrm::json::String>(hint_iter->second).value;
  return hint.c_str();
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

const char*
osrmc_trip_response_data_version(osrmc_trip_response_t response, osrmc_error_t* error) try {
  if (!osrmc_validate_trip_response(response, error)) {
    return nullptr;
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  const auto data_version_iter = response_typed->values.find("data_version");
  if (data_version_iter == response_typed->values.end()) {
    return nullptr;
  }

  const auto& data_version = std::get<osrm::json::String>(data_version_iter->second).value;
  return data_version.c_str();
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

osrmc_blob_t
osrmc_trip_response_json(osrmc_trip_response_t response, osrmc_error_t* error) try {
  if (!osrmc_validate_trip_response(response, error)) {
    return nullptr;
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);
  return osrmc_render_json(*response_typed);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

const char*
osrmc_trip_response_geometry_polyline(osrmc_trip_response_t response, unsigned trip_index, osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  if (response_typed->values.find("trips") == response_typed->values.end()) {
    *error = new osrmc_error{"NoTrips", "No trips available in response"};
    return nullptr;
  }

  const auto& trips = std::get<osrm::json::Array>(response_typed->values.at("trips"));
  if (trip_index >= trips.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Trip index out of bounds"};
    return nullptr;
  }

  const auto& trip = std::get<osrm::json::Object>(trips.values.at(trip_index));
  const auto geometry_iter = trip.values.find("geometry");
  if (geometry_iter == trip.values.end()) {
    *error = new osrmc_error{"NoGeometry", "Geometry not available for this trip"};
    return nullptr;
  }

  const auto& geometry = geometry_iter->second;
  if (std::holds_alternative<osrm::json::String>(geometry)) {
    const auto& polyline = std::get<osrm::json::String>(geometry).value;
    return polyline.c_str();
  }

  *error = new osrmc_error{"NoPolyline", "Geometry not available as polyline for this trip"};
  return nullptr;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

unsigned
osrmc_trip_response_geometry_coordinate_count(osrmc_trip_response_t response,
                                              unsigned trip_index,
                                              osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  if (response_typed->values.find("trips") == response_typed->values.end()) {
    if (error) {
      *error = new osrmc_error{"NoTrips", "No trips available in response"};
    }
    return 0;
  }

  const auto& trips = std::get<osrm::json::Array>(response_typed->values.at("trips"));
  if (trip_index >= trips.values.size()) {
    if (error) {
      *error = new osrmc_error{"IndexOutOfBounds", "Trip index out of bounds"};
    }
    return 0;
  }

  const auto& trip = std::get<osrm::json::Object>(trips.values.at(trip_index));
  const auto* coordinates = osrmc_get_route_geometry_coordinates(trip, error);
  if (!coordinates) {
    return 0;
  }
  return static_cast<unsigned>(coordinates->values.size());
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return 0;
}

double
osrmc_trip_response_geometry_coordinate_latitude(osrmc_trip_response_t response,
                                                 unsigned trip_index,
                                                 unsigned coord_index,
                                                 osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  if (response_typed->values.find("trips") == response_typed->values.end()) {
    *error = new osrmc_error{"NoTrips", "No trips available in response"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& trips = std::get<osrm::json::Array>(response_typed->values.at("trips"));
  if (trip_index >= trips.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Trip index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& trip = std::get<osrm::json::Object>(trips.values.at(trip_index));
  const auto* coordinates = osrmc_get_route_geometry_coordinates(trip, error);
  if (!coordinates) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  if (coord_index >= coordinates->values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Coordinate index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& coordinate_pair = std::get<osrm::json::Array>(coordinates->values.at(coord_index)).values;
  if (coordinate_pair.size() < MIN_COORDINATE_PAIR_SIZE) {
    *error = new osrmc_error{"InvalidGeometry", "Coordinate entry is malformed"};
    return std::numeric_limits<double>::quiet_NaN();
  }
  return std::get<osrm::json::Number>(coordinate_pair[COORDINATE_LATITUDE_INDEX]).value;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::quiet_NaN();
}

double
osrmc_trip_response_geometry_coordinate_longitude(osrmc_trip_response_t response,
                                                  unsigned trip_index,
                                                  unsigned coord_index,
                                                  osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  if (response_typed->values.find("trips") == response_typed->values.end()) {
    *error = new osrmc_error{"NoTrips", "No trips available in response"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& trips = std::get<osrm::json::Array>(response_typed->values.at("trips"));
  if (trip_index >= trips.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Trip index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& trip = std::get<osrm::json::Object>(trips.values.at(trip_index));
  const auto* coordinates = osrmc_get_route_geometry_coordinates(trip, error);
  if (!coordinates) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  if (coord_index >= coordinates->values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Coordinate index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& coordinate_pair = std::get<osrm::json::Array>(coordinates->values.at(coord_index)).values;
  if (coordinate_pair.size() < MIN_COORDINATE_PAIR_SIZE) {
    *error = new osrmc_error{"InvalidGeometry", "Coordinate entry is malformed"};
    return std::numeric_limits<double>::quiet_NaN();
  }
  return std::get<osrm::json::Number>(coordinate_pair[COORDINATE_LONGITUDE_INDEX]).value;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::quiet_NaN();
}

double
osrmc_trip_response_weight(osrmc_trip_response_t response, unsigned trip_index, osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  if (response_typed->values.find("trips") == response_typed->values.end()) {
    *error = new osrmc_error{"NoTrips", "No trips available in response"};
    return std::numeric_limits<double>::infinity();
  }

  const auto& trips = std::get<osrm::json::Array>(response_typed->values.at("trips"));
  if (trip_index >= trips.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Trip index out of bounds"};
    return std::numeric_limits<double>::infinity();
  }

  const auto& trip = std::get<osrm::json::Object>(trips.values.at(trip_index));
  const auto weight_iter = trip.values.find("weight");
  if (weight_iter == trip.values.end()) {
    *error = new osrmc_error{"NoWeight", "Weight not available for this trip"};
    return std::numeric_limits<double>::infinity();
  }

  const auto weight = std::get<osrm::json::Number>(weight_iter->second).value;
  return weight;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::infinity();
}

const char*
osrmc_trip_response_weight_name(osrmc_trip_response_t response, unsigned trip_index, osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  if (response_typed->values.find("trips") == response_typed->values.end()) {
    *error = new osrmc_error{"NoTrips", "No trips available in response"};
    return nullptr;
  }

  const auto& trips = std::get<osrm::json::Array>(response_typed->values.at("trips"));
  if (trip_index >= trips.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Trip index out of bounds"};
    return nullptr;
  }

  const auto& trip = std::get<osrm::json::Object>(trips.values.at(trip_index));
  const auto weight_name_iter = trip.values.find("weight_name");
  if (weight_name_iter == trip.values.end()) {
    *error = new osrmc_error{"NoWeightName", "Weight name not available for this trip"};
    return nullptr;
  }

  const auto& weight_name = std::get<osrm::json::String>(weight_name_iter->second).value;
  return weight_name.c_str();
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

unsigned
osrmc_trip_response_alternative_count(osrmc_trip_response_t response, osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  if (response_typed->values.find("trips") == response_typed->values.end()) {
    if (error) {
      *error = new osrmc_error{"NoTrips", "No trips available in response"};
    }
    return 0;
  }

  const auto& trips = std::get<osrm::json::Array>(response_typed->values.at("trips"));
  return static_cast<unsigned>(trips.values.size());
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return 0;
}

unsigned
osrmc_trip_response_leg_count(osrmc_trip_response_t response, unsigned trip_index, osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  if (response_typed->values.find("trips") == response_typed->values.end()) {
    *error = new osrmc_error{"NoTrips", "No trips available in response"};
    return 0;
  }

  const auto& trips = std::get<osrm::json::Array>(response_typed->values.at("trips"));
  if (trip_index >= trips.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Trip index out of bounds"};
    return 0;
  }

  const auto& trip = std::get<osrm::json::Object>(trips.values.at(trip_index));
  if (trip.values.find("legs") == trip.values.end()) {
    return 0;
  }

  const auto& legs = std::get<osrm::json::Array>(trip.values.at("legs"));
  return static_cast<unsigned>(legs.values.size());
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return 0;
}

double
osrmc_trip_response_leg_weight(osrmc_trip_response_t response,
                               unsigned trip_index,
                               unsigned leg_index,
                               osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  if (response_typed->values.find("trips") == response_typed->values.end()) {
    *error = new osrmc_error{"NoTrips", "No trips available in response"};
    return std::numeric_limits<double>::infinity();
  }

  const auto& trips = std::get<osrm::json::Array>(response_typed->values.at("trips"));
  if (trip_index >= trips.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Trip index out of bounds"};
    return std::numeric_limits<double>::infinity();
  }

  const auto& trip = std::get<osrm::json::Object>(trips.values.at(trip_index));
  const auto& legs = std::get<osrm::json::Array>(trip.values.at("legs"));
  if (leg_index >= legs.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Leg index out of bounds"};
    return std::numeric_limits<double>::infinity();
  }

  const auto& leg = std::get<osrm::json::Object>(legs.values.at(leg_index));
  const auto weight_iter = leg.values.find("weight");
  if (weight_iter == leg.values.end()) {
    *error = new osrmc_error{"NoWeight", "Weight not available for this leg"};
    return std::numeric_limits<double>::infinity();
  }

  const auto weight = std::get<osrm::json::Number>(weight_iter->second).value;
  return weight;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::infinity();
}

const char*
osrmc_trip_response_leg_summary(osrmc_trip_response_t response,
                                unsigned trip_index,
                                unsigned leg_index,
                                osrmc_error_t* error) try {
  if (!osrmc_validate_trip_response(response, error)) {
    return nullptr;
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);
  const auto* leg = osrmc_get_trip_leg(*response_typed, trip_index, leg_index, error);
  if (!leg) {
    return nullptr;
  }

  const auto summary_iter = leg->values.find("summary");
  if (summary_iter == leg->values.end()) {
    return nullptr;
  }

  const auto& summary = std::get<osrm::json::String>(summary_iter->second).value;
  return summary.c_str();
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

unsigned
osrmc_trip_response_leg_annotations_count(osrmc_trip_response_t response,
                                          unsigned trip_index,
                                          unsigned leg_index,
                                          osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  if (response_typed->values.find("trips") == response_typed->values.end()) {
    *error = new osrmc_error{"NoTrips", "No trips available in response"};
    return 0;
  }

  const auto& trips = std::get<osrm::json::Array>(response_typed->values.at("trips"));
  if (trip_index >= trips.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Trip index out of bounds"};
    return 0;
  }

  const auto& trip = std::get<osrm::json::Object>(trips.values.at(trip_index));
  const auto& legs = std::get<osrm::json::Array>(trip.values.at("legs"));
  if (leg_index >= legs.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Leg index out of bounds"};
    return 0;
  }

  const auto& leg = std::get<osrm::json::Object>(legs.values.at(leg_index));
  const auto annotations_iter = leg.values.find("annotation");
  if (annotations_iter == leg.values.end()) {
    return 0;
  }

  const auto& annotation = std::get<osrm::json::Object>(annotations_iter->second);

  if (annotation.values.find("duration") != annotation.values.end()) {
    const auto& duration_array = std::get<osrm::json::Array>(annotation.values.at("duration"));
    return static_cast<unsigned>(duration_array.values.size());
  }
  if (annotation.values.find("distance") != annotation.values.end()) {
    const auto& distance_array = std::get<osrm::json::Array>(annotation.values.at("distance"));
    return static_cast<unsigned>(distance_array.values.size());
  }
  if (annotation.values.find("speed") != annotation.values.end()) {
    const auto& speed_array = std::get<osrm::json::Array>(annotation.values.at("speed"));
    return static_cast<unsigned>(speed_array.values.size());
  }
  if (annotation.values.find("weight") != annotation.values.end()) {
    const auto& weight_array = std::get<osrm::json::Array>(annotation.values.at("weight"));
    return static_cast<unsigned>(weight_array.values.size());
  }
  if (annotation.values.find("nodes") != annotation.values.end()) {
    const auto& nodes_array = std::get<osrm::json::Array>(annotation.values.at("nodes"));
    return static_cast<unsigned>(nodes_array.values.size());
  }
  if (annotation.values.find("datasources") != annotation.values.end()) {
    const auto& datasources_array = std::get<osrm::json::Array>(annotation.values.at("datasources"));
    return static_cast<unsigned>(datasources_array.values.size());
  }

  return 0;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return 0;
}

double
osrmc_trip_response_leg_annotations_speed(osrmc_trip_response_t response,
                                          unsigned trip_index,
                                          unsigned leg_index,
                                          unsigned annotation_index,
                                          osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  if (response_typed->values.find("trips") == response_typed->values.end()) {
    *error = new osrmc_error{"NoTrips", "No trips available in response"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& trips = std::get<osrm::json::Array>(response_typed->values.at("trips"));
  if (trip_index >= trips.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Trip index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& trip = std::get<osrm::json::Object>(trips.values.at(trip_index));
  const auto& legs = std::get<osrm::json::Array>(trip.values.at("legs"));
  if (leg_index >= legs.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Leg index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& leg = std::get<osrm::json::Object>(legs.values.at(leg_index));
  const auto annotations_iter = leg.values.find("annotation");
  if (annotations_iter == leg.values.end()) {
    *error = new osrmc_error{"NoAnnotations", "Annotations not available for this leg"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& annotation = std::get<osrm::json::Object>(annotations_iter->second);
  const auto speed_iter = annotation.values.find("speed");
  if (speed_iter == annotation.values.end()) {
    *error = new osrmc_error{"NoSpeed", "Speed annotations not available"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& speed_array = std::get<osrm::json::Array>(speed_iter->second);
  if (annotation_index >= speed_array.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Annotation index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto speed = std::get<osrm::json::Number>(speed_array.values.at(annotation_index)).value;
  return speed;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::quiet_NaN();
}

double
osrmc_trip_response_leg_annotations_duration(osrmc_trip_response_t response,
                                             unsigned trip_index,
                                             unsigned leg_index,
                                             unsigned annotation_index,
                                             osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  if (response_typed->values.find("trips") == response_typed->values.end()) {
    *error = new osrmc_error{"NoTrips", "No trips available in response"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& trips = std::get<osrm::json::Array>(response_typed->values.at("trips"));
  if (trip_index >= trips.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Trip index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& trip = std::get<osrm::json::Object>(trips.values.at(trip_index));
  const auto& legs = std::get<osrm::json::Array>(trip.values.at("legs"));
  if (leg_index >= legs.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Leg index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& leg = std::get<osrm::json::Object>(legs.values.at(leg_index));
  const auto annotations_iter = leg.values.find("annotation");
  if (annotations_iter == leg.values.end()) {
    *error = new osrmc_error{"NoAnnotations", "Annotations not available for this leg"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& annotation = std::get<osrm::json::Object>(annotations_iter->second);
  const auto duration_iter = annotation.values.find("duration");
  if (duration_iter == annotation.values.end()) {
    *error = new osrmc_error{"NoDuration", "Duration annotations not available"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& duration_array = std::get<osrm::json::Array>(duration_iter->second);
  if (annotation_index >= duration_array.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Annotation index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto duration = std::get<osrm::json::Number>(duration_array.values.at(annotation_index)).value;
  return duration;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::quiet_NaN();
}

double
osrmc_trip_response_leg_annotations_distance(osrmc_trip_response_t response,
                                             unsigned trip_index,
                                             unsigned leg_index,
                                             unsigned annotation_index,
                                             osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  if (response_typed->values.find("trips") == response_typed->values.end()) {
    *error = new osrmc_error{"NoTrips", "No trips available in response"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& trips = std::get<osrm::json::Array>(response_typed->values.at("trips"));
  if (trip_index >= trips.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Trip index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& trip = std::get<osrm::json::Object>(trips.values.at(trip_index));
  const auto& legs = std::get<osrm::json::Array>(trip.values.at("legs"));
  if (leg_index >= legs.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Leg index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& leg = std::get<osrm::json::Object>(legs.values.at(leg_index));
  const auto annotations_iter = leg.values.find("annotation");
  if (annotations_iter == leg.values.end()) {
    *error = new osrmc_error{"NoAnnotations", "Annotations not available for this leg"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& annotation = std::get<osrm::json::Object>(annotations_iter->second);
  const auto distance_iter = annotation.values.find("distance");
  if (distance_iter == annotation.values.end()) {
    *error = new osrmc_error{"NoDistance", "Distance annotations not available"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& distance_array = std::get<osrm::json::Array>(distance_iter->second);
  if (annotation_index >= distance_array.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Annotation index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto distance = std::get<osrm::json::Number>(distance_array.values.at(annotation_index)).value;
  return distance;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::quiet_NaN();
}

double
osrmc_trip_response_leg_annotations_weight(osrmc_trip_response_t response,
                                           unsigned trip_index,
                                           unsigned leg_index,
                                           unsigned annotation_index,
                                           osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  if (response_typed->values.find("trips") == response_typed->values.end()) {
    *error = new osrmc_error{"NoTrips", "No trips available in response"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& trips = std::get<osrm::json::Array>(response_typed->values.at("trips"));
  if (trip_index >= trips.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Trip index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& trip = std::get<osrm::json::Object>(trips.values.at(trip_index));
  const auto& legs = std::get<osrm::json::Array>(trip.values.at("legs"));
  if (leg_index >= legs.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Leg index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& leg = std::get<osrm::json::Object>(legs.values.at(leg_index));
  const auto annotations_iter = leg.values.find("annotation");
  if (annotations_iter == leg.values.end()) {
    *error = new osrmc_error{"NoAnnotations", "Annotations not available for this leg"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& annotation = std::get<osrm::json::Object>(annotations_iter->second);
  const auto weight_iter = annotation.values.find("weight");
  if (weight_iter == annotation.values.end()) {
    *error = new osrmc_error{"NoWeight", "Weight annotations not available"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& weight_array = std::get<osrm::json::Array>(weight_iter->second);
  if (annotation_index >= weight_array.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Annotation index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto weight = std::get<osrm::json::Number>(weight_array.values.at(annotation_index)).value;
  return weight;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::quiet_NaN();
}

unsigned
osrmc_trip_response_leg_annotations_datasources(osrmc_trip_response_t response,
                                                unsigned trip_index,
                                                unsigned leg_index,
                                                unsigned annotation_index,
                                                osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  if (response_typed->values.find("trips") == response_typed->values.end()) {
    *error = new osrmc_error{"NoTrips", "No trips available in response"};
    return 0;
  }

  const auto& trips = std::get<osrm::json::Array>(response_typed->values.at("trips"));
  if (trip_index >= trips.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Trip index out of bounds"};
    return 0;
  }

  const auto& trip = std::get<osrm::json::Object>(trips.values.at(trip_index));
  const auto& legs = std::get<osrm::json::Array>(trip.values.at("legs"));
  if (leg_index >= legs.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Leg index out of bounds"};
    return 0;
  }

  const auto& leg = std::get<osrm::json::Object>(legs.values.at(leg_index));
  const auto annotations_iter = leg.values.find("annotation");
  if (annotations_iter == leg.values.end()) {
    *error = new osrmc_error{"NoAnnotations", "Annotations not available for this leg"};
    return 0;
  }

  const auto& annotation = std::get<osrm::json::Object>(annotations_iter->second);
  const auto datasources_iter = annotation.values.find("datasources");
  if (datasources_iter == annotation.values.end()) {
    *error = new osrmc_error{"NoDatasources", "Datasources annotations not available"};
    return 0;
  }

  const auto& datasources_array = std::get<osrm::json::Array>(datasources_iter->second);
  if (annotation_index >= datasources_array.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Annotation index out of bounds"};
    return 0;
  }

  const auto datasource = std::get<osrm::json::Number>(datasources_array.values.at(annotation_index)).value;
  return static_cast<unsigned>(datasource);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return 0;
}

unsigned long long
osrmc_trip_response_leg_annotations_nodes(osrmc_trip_response_t response,
                                          unsigned trip_index,
                                          unsigned leg_index,
                                          unsigned annotation_index,
                                          osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  if (response_typed->values.find("trips") == response_typed->values.end()) {
    *error = new osrmc_error{"NoTrips", "No trips available in response"};
    return 0;
  }

  const auto& trips = std::get<osrm::json::Array>(response_typed->values.at("trips"));
  if (trip_index >= trips.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Trip index out of bounds"};
    return 0;
  }

  const auto& trip = std::get<osrm::json::Object>(trips.values.at(trip_index));
  const auto& legs = std::get<osrm::json::Array>(trip.values.at("legs"));
  if (leg_index >= legs.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Leg index out of bounds"};
    return 0;
  }

  const auto& leg = std::get<osrm::json::Object>(legs.values.at(leg_index));
  const auto annotations_iter = leg.values.find("annotation");
  if (annotations_iter == leg.values.end()) {
    *error = new osrmc_error{"NoAnnotations", "Annotations not available for this leg"};
    return 0;
  }

  const auto& annotation = std::get<osrm::json::Object>(annotations_iter->second);
  const auto nodes_iter = annotation.values.find("nodes");
  if (nodes_iter == annotation.values.end()) {
    *error = new osrmc_error{"NoNodes", "Nodes annotations not available"};
    return 0;
  }

  const auto& nodes_array = std::get<osrm::json::Array>(nodes_iter->second);
  if (annotation_index >= nodes_array.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Annotation index out of bounds"};
    return 0;
  }

  const auto node = std::get<osrm::json::Number>(nodes_array.values.at(annotation_index)).value;
  return static_cast<unsigned long long>(node);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return 0;
}

unsigned
osrmc_trip_response_step_count(osrmc_trip_response_t response,
                               unsigned trip_index,
                               unsigned leg_index,
                               osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  if (response_typed->values.find("trips") == response_typed->values.end()) {
    *error = new osrmc_error{"NoTrips", "No trips available in response"};
    return 0;
  }

  const auto& trips = std::get<osrm::json::Array>(response_typed->values.at("trips"));
  if (trip_index >= trips.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Trip index out of bounds"};
    return 0;
  }

  const auto& trip = std::get<osrm::json::Object>(trips.values.at(trip_index));
  const auto& legs = std::get<osrm::json::Array>(trip.values.at("legs"));
  if (leg_index >= legs.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Leg index out of bounds"};
    return 0;
  }

  const auto& leg = std::get<osrm::json::Object>(legs.values.at(leg_index));
  if (leg.values.find("steps") == leg.values.end()) {
    return 0;
  }

  const auto& steps = std::get<osrm::json::Array>(leg.values.at("steps"));
  return static_cast<unsigned>(steps.values.size());
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return 0;
}

double
osrmc_trip_response_step_distance(osrmc_trip_response_t response,
                                  unsigned trip_index,
                                  unsigned leg_index,
                                  unsigned step_index,
                                  osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  if (response_typed->values.find("trips") == response_typed->values.end()) {
    *error = new osrmc_error{"NoTrips", "No trips available in response"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& trips = std::get<osrm::json::Array>(response_typed->values.at("trips"));
  if (trip_index >= trips.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Trip index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& trip = std::get<osrm::json::Object>(trips.values.at(trip_index));
  const auto& legs = std::get<osrm::json::Array>(trip.values.at("legs"));
  if (leg_index >= legs.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Leg index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& leg = std::get<osrm::json::Object>(legs.values.at(leg_index));
  const auto& steps = std::get<osrm::json::Array>(leg.values.at("steps"));
  if (step_index >= steps.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Step index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& step = std::get<osrm::json::Object>(steps.values.at(step_index));
  const auto distance = std::get<osrm::json::Number>(step.values.at("distance")).value;

  return distance;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::quiet_NaN();
}

double
osrmc_trip_response_step_duration(osrmc_trip_response_t response,
                                  unsigned trip_index,
                                  unsigned leg_index,
                                  unsigned step_index,
                                  osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  if (response_typed->values.find("trips") == response_typed->values.end()) {
    *error = new osrmc_error{"NoTrips", "No trips available in response"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& trips = std::get<osrm::json::Array>(response_typed->values.at("trips"));
  if (trip_index >= trips.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Trip index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& trip = std::get<osrm::json::Object>(trips.values.at(trip_index));
  const auto& legs = std::get<osrm::json::Array>(trip.values.at("legs"));
  if (leg_index >= legs.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Leg index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& leg = std::get<osrm::json::Object>(legs.values.at(leg_index));
  const auto& steps = std::get<osrm::json::Array>(leg.values.at("steps"));
  if (step_index >= steps.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Step index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& step = std::get<osrm::json::Object>(steps.values.at(step_index));
  const auto duration = std::get<osrm::json::Number>(step.values.at("duration")).value;

  return duration;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::quiet_NaN();
}

double
osrmc_trip_response_step_weight(osrmc_trip_response_t response,
                                unsigned trip_index,
                                unsigned leg_index,
                                unsigned step_index,
                                osrmc_error_t* error) try {
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);

  if (response_typed->values.find("trips") == response_typed->values.end()) {
    *error = new osrmc_error{"NoTrips", "No trips available in response"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& trips = std::get<osrm::json::Array>(response_typed->values.at("trips"));
  if (trip_index >= trips.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Trip index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& trip = std::get<osrm::json::Object>(trips.values.at(trip_index));
  const auto& legs = std::get<osrm::json::Array>(trip.values.at("legs"));
  if (leg_index >= legs.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Leg index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& leg = std::get<osrm::json::Object>(legs.values.at(leg_index));
  const auto& steps = std::get<osrm::json::Array>(leg.values.at("steps"));
  if (step_index >= steps.values.size()) {
    *error = new osrmc_error{"IndexOutOfBounds", "Step index out of bounds"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& step = std::get<osrm::json::Object>(steps.values.at(step_index));
  const auto weight_iter = step.values.find("weight");
  if (weight_iter == step.values.end()) {
    *error = new osrmc_error{"NoWeight", "Weight not available for this step"};
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto weight = std::get<osrm::json::Number>(weight_iter->second).value;
  return weight;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::quiet_NaN();
}

const char*
osrmc_trip_response_step_name(osrmc_trip_response_t response,
                              unsigned trip_index,
                              unsigned leg_index,
                              unsigned step_index,
                              osrmc_error_t* error) try {
  if (!osrmc_validate_trip_response(response, error)) {
    return nullptr;
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);
  const auto* step = osrmc_get_trip_step(*response_typed, trip_index, leg_index, step_index, error);
  if (!step) {
    return nullptr;
  }

  const auto name_iter = step->values.find("name");
  if (name_iter == step->values.end()) {
    if (error) {
      *error = new osrmc_error{"NoName", "Name not available for this step"};
    }
    return nullptr;
  }

  const auto& name = std::get<osrm::json::String>(name_iter->second).value;
  return name.c_str();
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

const char*
osrmc_trip_response_step_ref(osrmc_trip_response_t response,
                             unsigned trip_index,
                             unsigned leg_index,
                             unsigned step_index,
                             osrmc_error_t* error) try {
  if (!osrmc_validate_trip_response(response, error)) {
    return nullptr;
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);
  const auto* step = osrmc_get_trip_step(*response_typed, trip_index, leg_index, step_index, error);
  if (!step) {
    return nullptr;
  }

  const auto ref_iter = step->values.find("ref");
  if (ref_iter == step->values.end()) {
    return nullptr;
  }

  const auto& ref = std::get<osrm::json::String>(ref_iter->second).value;
  return ref.c_str();
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

const char*
osrmc_trip_response_step_pronunciation(osrmc_trip_response_t response,
                                       unsigned trip_index,
                                       unsigned leg_index,
                                       unsigned step_index,
                                       osrmc_error_t* error) try {
  if (!osrmc_validate_trip_response(response, error)) {
    return nullptr;
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);
  const auto* step = osrmc_get_trip_step(*response_typed, trip_index, leg_index, step_index, error);
  if (!step) {
    return nullptr;
  }

  const auto pronunciation_iter = step->values.find("pronunciation");
  if (pronunciation_iter == step->values.end()) {
    return nullptr;
  }

  const auto& pronunciation = std::get<osrm::json::String>(pronunciation_iter->second).value;
  return pronunciation.c_str();
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

const char*
osrmc_trip_response_step_destinations(osrmc_trip_response_t response,
                                      unsigned trip_index,
                                      unsigned leg_index,
                                      unsigned step_index,
                                      osrmc_error_t* error) try {
  if (!osrmc_validate_trip_response(response, error)) {
    return nullptr;
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);
  const auto* step = osrmc_get_trip_step(*response_typed, trip_index, leg_index, step_index, error);
  if (!step) {
    return nullptr;
  }

  const auto destinations_iter = step->values.find("destinations");
  if (destinations_iter == step->values.end()) {
    return nullptr;
  }

  const auto& destinations = std::get<osrm::json::String>(destinations_iter->second).value;
  return destinations.c_str();
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

const char*
osrmc_trip_response_step_exits(osrmc_trip_response_t response,
                               unsigned trip_index,
                               unsigned leg_index,
                               unsigned step_index,
                               osrmc_error_t* error) try {
  if (!osrmc_validate_trip_response(response, error)) {
    return nullptr;
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);
  const auto* step = osrmc_get_trip_step(*response_typed, trip_index, leg_index, step_index, error);
  if (!step) {
    return nullptr;
  }

  const auto exits_iter = step->values.find("exits");
  if (exits_iter == step->values.end()) {
    return nullptr;
  }

  const auto& exits = std::get<osrm::json::String>(exits_iter->second).value;
  return exits.c_str();
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

const char*
osrmc_trip_response_step_rotary_name(osrmc_trip_response_t response,
                                     unsigned trip_index,
                                     unsigned leg_index,
                                     unsigned step_index,
                                     osrmc_error_t* error) try {
  if (!osrmc_validate_trip_response(response, error)) {
    return nullptr;
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);
  const auto* step = osrmc_get_trip_step(*response_typed, trip_index, leg_index, step_index, error);
  if (!step) {
    return nullptr;
  }

  const auto rotary_name_iter = step->values.find("rotary_name");
  if (rotary_name_iter == step->values.end()) {
    return nullptr;
  }

  const auto& rotary_name = std::get<osrm::json::String>(rotary_name_iter->second).value;
  return rotary_name.c_str();
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

const char*
osrmc_trip_response_step_rotary_pronunciation(osrmc_trip_response_t response,
                                              unsigned trip_index,
                                              unsigned leg_index,
                                              unsigned step_index,
                                              osrmc_error_t* error) try {
  if (!osrmc_validate_trip_response(response, error)) {
    return nullptr;
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);
  const auto* step = osrmc_get_trip_step(*response_typed, trip_index, leg_index, step_index, error);
  if (!step) {
    return nullptr;
  }

  const auto rotary_pronunciation_iter = step->values.find("rotary_pronunciation");
  if (rotary_pronunciation_iter == step->values.end()) {
    return nullptr;
  }

  const auto& rotary_pronunciation = std::get<osrm::json::String>(rotary_pronunciation_iter->second).value;
  return rotary_pronunciation.c_str();
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

const char*
osrmc_trip_response_step_mode(osrmc_trip_response_t response,
                              unsigned trip_index,
                              unsigned leg_index,
                              unsigned step_index,
                              osrmc_error_t* error) try {
  if (!osrmc_validate_trip_response(response, error)) {
    return nullptr;
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);
  const auto* step = osrmc_get_trip_step(*response_typed, trip_index, leg_index, step_index, error);
  if (!step) {
    return nullptr;
  }

  const auto mode_iter = step->values.find("mode");
  if (mode_iter == step->values.end()) {
    if (error) {
      *error = new osrmc_error{"NoMode", "Mode not available for this step"};
    }
    return nullptr;
  }

  const auto& mode = std::get<osrm::json::String>(mode_iter->second).value;
  return mode.c_str();
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

const char*
osrmc_trip_response_step_driving_side(osrmc_trip_response_t response,
                                      unsigned trip_index,
                                      unsigned leg_index,
                                      unsigned step_index,
                                      osrmc_error_t* error) try {
  if (!osrmc_validate_trip_response(response, error)) {
    return nullptr;
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);
  const auto* step = osrmc_get_trip_step(*response_typed, trip_index, leg_index, step_index, error);
  if (!step) {
    return nullptr;
  }

  const auto driving_side_iter = step->values.find("driving_side");
  if (driving_side_iter == step->values.end()) {
    if (error) {
      *error = new osrmc_error{"NoDrivingSide", "Driving side not available for this step"};
    }
    return nullptr;
  }

  const auto& driving_side = std::get<osrm::json::String>(driving_side_iter->second).value;
  return driving_side.c_str();
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

const char*
osrmc_trip_response_step_maneuver_type(osrmc_trip_response_t response,
                                       unsigned trip_index,
                                       unsigned leg_index,
                                       unsigned step_index,
                                       osrmc_error_t* error) try {
  if (!osrmc_validate_trip_response(response, error)) {
    return nullptr;
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);
  const auto* step = osrmc_get_trip_step(*response_typed, trip_index, leg_index, step_index, error);
  if (!step) {
    return nullptr;
  }

  const auto maneuver_iter = step->values.find("maneuver");
  if (maneuver_iter == step->values.end()) {
    if (error) {
      *error = new osrmc_error{"NoManeuver", "Maneuver not available for this step"};
    }
    return nullptr;
  }

  const auto& maneuver = std::get<osrm::json::Object>(maneuver_iter->second);
  const auto type_iter = maneuver.values.find("type");
  if (type_iter == maneuver.values.end()) {
    if (error) {
      *error = new osrmc_error{"NoType", "Type not available for this maneuver"};
    }
    return nullptr;
  }

  const auto& type = std::get<osrm::json::String>(type_iter->second).value;
  return type.c_str();
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

const char*
osrmc_trip_response_step_maneuver_modifier(osrmc_trip_response_t response,
                                           unsigned trip_index,
                                           unsigned leg_index,
                                           unsigned step_index,
                                           osrmc_error_t* error) try {
  if (!osrmc_validate_trip_response(response, error)) {
    return nullptr;
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);
  const auto* step = osrmc_get_trip_step(*response_typed, trip_index, leg_index, step_index, error);
  if (!step) {
    return nullptr;
  }

  const auto maneuver_iter = step->values.find("maneuver");
  if (maneuver_iter == step->values.end()) {
    if (error) {
      *error = new osrmc_error{"NoManeuver", "Maneuver not available for this step"};
    }
    return nullptr;
  }

  const auto& maneuver = std::get<osrm::json::Object>(maneuver_iter->second);
  const auto modifier_iter = maneuver.values.find("modifier");
  if (modifier_iter == maneuver.values.end()) {
    return nullptr;
  }

  const auto& modifier = std::get<osrm::json::String>(modifier_iter->second).value;
  return modifier.c_str();
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

double
osrmc_trip_response_step_maneuver_location_latitude(osrmc_trip_response_t response,
                                                    unsigned trip_index,
                                                    unsigned leg_index,
                                                    unsigned step_index,
                                                    osrmc_error_t* error) try {
  if (!osrmc_validate_trip_response(response, error)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);
  const auto* step = osrmc_get_trip_step(*response_typed, trip_index, leg_index, step_index, error);
  if (!step) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto maneuver_iter = step->values.find("maneuver");
  if (maneuver_iter == step->values.end()) {
    if (error) {
      *error = new osrmc_error{"NoManeuver", "Maneuver not available for this step"};
    }
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& maneuver = std::get<osrm::json::Object>(maneuver_iter->second);
  const auto location_iter = maneuver.values.find("location");
  if (location_iter == maneuver.values.end()) {
    if (error) {
      *error = new osrmc_error{"NoLocation", "Location not available for this maneuver"};
    }
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& location = std::get<osrm::json::Array>(location_iter->second);
  if (location.values.size() < 2) {
    if (error) {
      *error = new osrmc_error{"InvalidLocation", "Location array does not have enough elements"};
    }
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto latitude = std::get<osrm::json::Number>(location.values[COORDINATE_LATITUDE_INDEX]).value;
  return latitude;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::quiet_NaN();
}

double
osrmc_trip_response_step_maneuver_location_longitude(osrmc_trip_response_t response,
                                                     unsigned trip_index,
                                                     unsigned leg_index,
                                                     unsigned step_index,
                                                     osrmc_error_t* error) try {
  if (!osrmc_validate_trip_response(response, error)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);
  const auto* step = osrmc_get_trip_step(*response_typed, trip_index, leg_index, step_index, error);
  if (!step) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto maneuver_iter = step->values.find("maneuver");
  if (maneuver_iter == step->values.end()) {
    if (error) {
      *error = new osrmc_error{"NoManeuver", "Maneuver not available for this step"};
    }
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& maneuver = std::get<osrm::json::Object>(maneuver_iter->second);
  const auto location_iter = maneuver.values.find("location");
  if (location_iter == maneuver.values.end()) {
    if (error) {
      *error = new osrmc_error{"NoLocation", "Location not available for this maneuver"};
    }
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& location = std::get<osrm::json::Array>(location_iter->second);
  if (location.values.size() < 2) {
    if (error) {
      *error = new osrmc_error{"InvalidLocation", "Location array does not have enough elements"};
    }
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto longitude = std::get<osrm::json::Number>(location.values[COORDINATE_LONGITUDE_INDEX]).value;
  return longitude;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::quiet_NaN();
}

double
osrmc_trip_response_step_maneuver_bearing_before(osrmc_trip_response_t response,
                                                 unsigned trip_index,
                                                 unsigned leg_index,
                                                 unsigned step_index,
                                                 osrmc_error_t* error) try {
  if (!osrmc_validate_trip_response(response, error)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);
  const auto* step = osrmc_get_trip_step(*response_typed, trip_index, leg_index, step_index, error);
  if (!step) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto maneuver_iter = step->values.find("maneuver");
  if (maneuver_iter == step->values.end()) {
    if (error) {
      *error = new osrmc_error{"NoManeuver", "Maneuver not available for this step"};
    }
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& maneuver = std::get<osrm::json::Object>(maneuver_iter->second);
  const auto bearing_before_iter = maneuver.values.find("bearing_before");
  if (bearing_before_iter == maneuver.values.end()) {
    if (error) {
      *error = new osrmc_error{"NoBearingBefore", "Bearing before not available for this maneuver"};
    }
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto bearing_before = std::get<osrm::json::Number>(bearing_before_iter->second).value;
  return bearing_before;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::quiet_NaN();
}

double
osrmc_trip_response_step_maneuver_bearing_after(osrmc_trip_response_t response,
                                                unsigned trip_index,
                                                unsigned leg_index,
                                                unsigned step_index,
                                                osrmc_error_t* error) try {
  if (!osrmc_validate_trip_response(response, error)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);
  const auto* step = osrmc_get_trip_step(*response_typed, trip_index, leg_index, step_index, error);
  if (!step) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto maneuver_iter = step->values.find("maneuver");
  if (maneuver_iter == step->values.end()) {
    if (error) {
      *error = new osrmc_error{"NoManeuver", "Maneuver not available for this step"};
    }
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& maneuver = std::get<osrm::json::Object>(maneuver_iter->second);
  const auto bearing_after_iter = maneuver.values.find("bearing_after");
  if (bearing_after_iter == maneuver.values.end()) {
    if (error) {
      *error = new osrmc_error{"NoBearingAfter", "Bearing after not available for this maneuver"};
    }
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto bearing_after = std::get<osrm::json::Number>(bearing_after_iter->second).value;
  return bearing_after;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::quiet_NaN();
}

int
osrmc_trip_response_step_maneuver_exit(osrmc_trip_response_t response,
                                       unsigned trip_index,
                                       unsigned leg_index,
                                       unsigned step_index,
                                       osrmc_error_t* error) try {
  if (!osrmc_validate_trip_response(response, error)) {
    return -1;
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);
  const auto* step = osrmc_get_trip_step(*response_typed, trip_index, leg_index, step_index, error);
  if (!step) {
    return -1;
  }

  const auto maneuver_iter = step->values.find("maneuver");
  if (maneuver_iter == step->values.end()) {
    if (error) {
      *error = new osrmc_error{"NoManeuver", "Maneuver not available for this step"};
    }
    return -1;
  }

  const auto& maneuver = std::get<osrm::json::Object>(maneuver_iter->second);
  const auto exit_iter = maneuver.values.find("exit");
  if (exit_iter == maneuver.values.end()) {
    return -1;
  }

  const auto exit = std::get<osrm::json::Number>(exit_iter->second).value;
  return static_cast<int>(exit);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return -1;
}

const char*
osrmc_trip_response_step_geometry_polyline(osrmc_trip_response_t response,
                                           unsigned trip_index,
                                           unsigned leg_index,
                                           unsigned step_index,
                                           osrmc_error_t* error) try {
  if (!osrmc_validate_trip_response(response, error)) {
    return nullptr;
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);
  const auto* step = osrmc_get_trip_step(*response_typed, trip_index, leg_index, step_index, error);
  if (!step) {
    return nullptr;
  }

  const auto geometry_iter = step->values.find("geometry");
  if (geometry_iter == step->values.end()) {
    if (error) {
      *error = new osrmc_error{"NoGeometry", "Geometry not available for this step"};
    }
    return nullptr;
  }

  const auto& geometry = geometry_iter->second;

  if (std::holds_alternative<osrm::json::String>(geometry)) {
    const auto& polyline = std::get<osrm::json::String>(geometry).value;
    return polyline.c_str();
  }

  if (error) {
    *error = new osrmc_error{"UnsupportedGeometry", "Step geometry is GeoJSON, not polyline"};
  }
  return nullptr;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

unsigned
osrmc_trip_response_step_geometry_coordinate_count(osrmc_trip_response_t response,
                                                   unsigned trip_index,
                                                   unsigned leg_index,
                                                   unsigned step_index,
                                                   osrmc_error_t* error) try {
  if (!osrmc_validate_trip_response(response, error)) {
    return 0;
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);
  const auto* step = osrmc_get_trip_step(*response_typed, trip_index, leg_index, step_index, error);
  if (!step) {
    return 0;
  }

  const auto* coordinates = osrmc_get_step_geometry_coordinates(*step, error);
  if (!coordinates) {
    return 0;
  }

  return static_cast<unsigned>(coordinates->values.size());
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return 0;
}

double
osrmc_trip_response_step_geometry_coordinate_latitude(osrmc_trip_response_t response,
                                                      unsigned trip_index,
                                                      unsigned leg_index,
                                                      unsigned step_index,
                                                      unsigned coord_index,
                                                      osrmc_error_t* error) try {
  if (!osrmc_validate_trip_response(response, error)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);
  const auto* step = osrmc_get_trip_step(*response_typed, trip_index, leg_index, step_index, error);
  if (!step) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto* coordinates = osrmc_get_step_geometry_coordinates(*step, error);
  if (!coordinates) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  if (coord_index >= coordinates->values.size()) {
    if (error) {
      *error = new osrmc_error{"IndexOutOfBounds", "Coordinate index out of bounds"};
    }
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& coord = std::get<osrm::json::Array>(coordinates->values.at(coord_index));
  if (coord.values.size() < 2) {
    if (error) {
      *error = new osrmc_error{"InvalidCoordinate", "Coordinate array does not have enough elements"};
    }
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto latitude = std::get<osrm::json::Number>(coord.values[COORDINATE_LATITUDE_INDEX]).value;
  return latitude;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::quiet_NaN();
}

double
osrmc_trip_response_step_geometry_coordinate_longitude(osrmc_trip_response_t response,
                                                       unsigned trip_index,
                                                       unsigned leg_index,
                                                       unsigned step_index,
                                                       unsigned coord_index,
                                                       osrmc_error_t* error) try {
  if (!osrmc_validate_trip_response(response, error)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);
  const auto* step = osrmc_get_trip_step(*response_typed, trip_index, leg_index, step_index, error);
  if (!step) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto* coordinates = osrmc_get_step_geometry_coordinates(*step, error);
  if (!coordinates) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  if (coord_index >= coordinates->values.size()) {
    if (error) {
      *error = new osrmc_error{"IndexOutOfBounds", "Coordinate index out of bounds"};
    }
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& coord = std::get<osrm::json::Array>(coordinates->values.at(coord_index));
  if (coord.values.size() < 2) {
    if (error) {
      *error = new osrmc_error{"InvalidCoordinate", "Coordinate array does not have enough elements"};
    }
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto longitude = std::get<osrm::json::Number>(coord.values[COORDINATE_LONGITUDE_INDEX]).value;
  return longitude;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::quiet_NaN();
}

unsigned
osrmc_trip_response_step_intersection_count(osrmc_trip_response_t response,
                                            unsigned trip_index,
                                            unsigned leg_index,
                                            unsigned step_index,
                                            osrmc_error_t* error) try {
  if (!osrmc_validate_trip_response(response, error)) {
    return 0;
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);
  const auto* step = osrmc_get_trip_step(*response_typed, trip_index, leg_index, step_index, error);
  if (!step) {
    return 0;
  }

  const auto intersections_iter = step->values.find("intersections");
  if (intersections_iter == step->values.end()) {
    return 0;
  }

  const auto& intersections = std::get<osrm::json::Array>(intersections_iter->second);
  return static_cast<unsigned>(intersections.values.size());
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return 0;
}

double
osrmc_trip_response_step_intersection_location_latitude(osrmc_trip_response_t response,
                                                        unsigned trip_index,
                                                        unsigned leg_index,
                                                        unsigned step_index,
                                                        unsigned intersection_index,
                                                        osrmc_error_t* error) try {
  if (!osrmc_validate_trip_response(response, error)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);
  const auto* step = osrmc_get_trip_step(*response_typed, trip_index, leg_index, step_index, error);
  if (!step) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto* intersection = osrmc_get_step_intersection(*step, intersection_index, error);
  if (!intersection) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto location_iter = intersection->values.find("location");
  if (location_iter == intersection->values.end()) {
    if (error) {
      *error = new osrmc_error{"NoLocation", "Location not available for this intersection"};
    }
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& location = std::get<osrm::json::Array>(location_iter->second);
  if (location.values.size() < 2) {
    if (error) {
      *error = new osrmc_error{"InvalidLocation", "Location array does not have enough elements"};
    }
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto latitude = std::get<osrm::json::Number>(location.values[COORDINATE_LATITUDE_INDEX]).value;
  return latitude;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::quiet_NaN();
}

double
osrmc_trip_response_step_intersection_location_longitude(osrmc_trip_response_t response,
                                                         unsigned trip_index,
                                                         unsigned leg_index,
                                                         unsigned step_index,
                                                         unsigned intersection_index,
                                                         osrmc_error_t* error) try {
  if (!osrmc_validate_trip_response(response, error)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);
  const auto* step = osrmc_get_trip_step(*response_typed, trip_index, leg_index, step_index, error);
  if (!step) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto* intersection = osrmc_get_step_intersection(*step, intersection_index, error);
  if (!intersection) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto location_iter = intersection->values.find("location");
  if (location_iter == intersection->values.end()) {
    if (error) {
      *error = new osrmc_error{"NoLocation", "Location not available for this intersection"};
    }
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& location = std::get<osrm::json::Array>(location_iter->second);
  if (location.values.size() < 2) {
    if (error) {
      *error = new osrmc_error{"InvalidLocation", "Location array does not have enough elements"};
    }
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto longitude = std::get<osrm::json::Number>(location.values[COORDINATE_LONGITUDE_INDEX]).value;
  return longitude;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::quiet_NaN();
}

unsigned
osrmc_trip_response_step_intersection_bearings_count(osrmc_trip_response_t response,
                                                     unsigned trip_index,
                                                     unsigned leg_index,
                                                     unsigned step_index,
                                                     unsigned intersection_index,
                                                     osrmc_error_t* error) try {
  if (!osrmc_validate_trip_response(response, error)) {
    return 0;
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);
  const auto* step = osrmc_get_trip_step(*response_typed, trip_index, leg_index, step_index, error);
  if (!step) {
    return 0;
  }

  const auto* intersection = osrmc_get_step_intersection(*step, intersection_index, error);
  if (!intersection) {
    return 0;
  }

  const auto bearings_iter = intersection->values.find("bearings");
  if (bearings_iter == intersection->values.end()) {
    return 0;
  }

  const auto& bearings = std::get<osrm::json::Array>(bearings_iter->second);
  return static_cast<unsigned>(bearings.values.size());
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return 0;
}

double
osrmc_trip_response_step_intersection_bearing(osrmc_trip_response_t response,
                                              unsigned trip_index,
                                              unsigned leg_index,
                                              unsigned step_index,
                                              unsigned intersection_index,
                                              unsigned bearing_index,
                                              osrmc_error_t* error) try {
  if (!osrmc_validate_trip_response(response, error)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);
  const auto* step = osrmc_get_trip_step(*response_typed, trip_index, leg_index, step_index, error);
  if (!step) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto* intersection = osrmc_get_step_intersection(*step, intersection_index, error);
  if (!intersection) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto bearings_iter = intersection->values.find("bearings");
  if (bearings_iter == intersection->values.end()) {
    if (error) {
      *error = new osrmc_error{"NoBearings", "Bearings not available for this intersection"};
    }
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto& bearings = std::get<osrm::json::Array>(bearings_iter->second);
  if (bearing_index >= bearings.values.size()) {
    if (error) {
      *error = new osrmc_error{"IndexOutOfBounds", "Bearing index out of bounds"};
    }
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto bearing = std::get<osrm::json::Number>(bearings.values.at(bearing_index)).value;
  return bearing;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::quiet_NaN();
}

int
osrmc_trip_response_step_intersection_entry(osrmc_trip_response_t response,
                                            unsigned trip_index,
                                            unsigned leg_index,
                                            unsigned step_index,
                                            unsigned intersection_index,
                                            unsigned entry_index,
                                            osrmc_error_t* error) try {
  if (!osrmc_validate_trip_response(response, error)) {
    return -1;
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);
  const auto* step = osrmc_get_trip_step(*response_typed, trip_index, leg_index, step_index, error);
  if (!step) {
    return -1;
  }

  const auto* intersection = osrmc_get_step_intersection(*step, intersection_index, error);
  if (!intersection) {
    return -1;
  }

  const auto entry_iter = intersection->values.find("entry");
  if (entry_iter == intersection->values.end()) {
    if (error) {
      *error = new osrmc_error{"NoEntry", "Entry flags not available for this intersection"};
    }
    return -1;
  }

  const auto& entry = std::get<osrm::json::Array>(entry_iter->second);
  if (entry_index >= entry.values.size()) {
    if (error) {
      *error = new osrmc_error{"IndexOutOfBounds", "Entry index out of bounds"};
    }
    return -1;
  }

  const auto& entry_value = entry.values.at(entry_index);
  if (std::holds_alternative<osrm::json::True>(entry_value)) {
    return 1;
  } else if (std::holds_alternative<osrm::json::False>(entry_value)) {
    return 0;
  } else {
    if (error) {
      *error = new osrmc_error{"InvalidEntry", "Entry value is not a boolean"};
    }
    return -1;
  }
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return -1;
}

double
osrmc_trip_response_step_intersection_in_bearing(osrmc_trip_response_t response,
                                                 unsigned trip_index,
                                                 unsigned leg_index,
                                                 unsigned step_index,
                                                 unsigned intersection_index,
                                                 osrmc_error_t* error) try {
  if (!osrmc_validate_trip_response(response, error)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);
  const auto* step = osrmc_get_trip_step(*response_typed, trip_index, leg_index, step_index, error);
  if (!step) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto* intersection = osrmc_get_step_intersection(*step, intersection_index, error);
  if (!intersection) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto in_bearing_iter = intersection->values.find("in");
  if (in_bearing_iter == intersection->values.end()) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto in_bearing = std::get<osrm::json::Number>(in_bearing_iter->second).value;
  return in_bearing;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::quiet_NaN();
}

double
osrmc_trip_response_step_intersection_out_bearing(osrmc_trip_response_t response,
                                                  unsigned trip_index,
                                                  unsigned leg_index,
                                                  unsigned step_index,
                                                  unsigned intersection_index,
                                                  osrmc_error_t* error) try {
  if (!osrmc_validate_trip_response(response, error)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);
  const auto* step = osrmc_get_trip_step(*response_typed, trip_index, leg_index, step_index, error);
  if (!step) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto* intersection = osrmc_get_step_intersection(*step, intersection_index, error);
  if (!intersection) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto out_bearing_iter = intersection->values.find("out");
  if (out_bearing_iter == intersection->values.end()) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto out_bearing = std::get<osrm::json::Number>(out_bearing_iter->second).value;
  return out_bearing;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return std::numeric_limits<double>::quiet_NaN();
}

unsigned
osrmc_trip_response_step_intersection_lanes_count(osrmc_trip_response_t response,
                                                  unsigned trip_index,
                                                  unsigned leg_index,
                                                  unsigned step_index,
                                                  unsigned intersection_index,
                                                  osrmc_error_t* error) try {
  if (!osrmc_validate_trip_response(response, error)) {
    return 0;
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);
  const auto* step = osrmc_get_trip_step(*response_typed, trip_index, leg_index, step_index, error);
  if (!step) {
    return 0;
  }

  const auto* intersection = osrmc_get_step_intersection(*step, intersection_index, error);
  if (!intersection) {
    return 0;
  }

  const auto lanes_iter = intersection->values.find("lanes");
  if (lanes_iter == intersection->values.end()) {
    return 0;
  }

  const auto& lanes = std::get<osrm::json::Array>(lanes_iter->second);
  return static_cast<unsigned>(lanes.values.size());
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return 0;
}

unsigned
osrmc_trip_response_step_intersection_lane_indications_count(osrmc_trip_response_t response,
                                                             unsigned trip_index,
                                                             unsigned leg_index,
                                                             unsigned step_index,
                                                             unsigned intersection_index,
                                                             unsigned lane_index,
                                                             osrmc_error_t* error) try {
  if (!osrmc_validate_trip_response(response, error)) {
    return 0;
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);
  const auto* step = osrmc_get_trip_step(*response_typed, trip_index, leg_index, step_index, error);
  if (!step) {
    return 0;
  }

  const auto* intersection = osrmc_get_step_intersection(*step, intersection_index, error);
  if (!intersection) {
    return 0;
  }

  const auto lanes_iter = intersection->values.find("lanes");
  if (lanes_iter == intersection->values.end()) {
    if (error) {
      *error = new osrmc_error{"NoLanes", "Lanes not available for this intersection"};
    }
    return 0;
  }

  const auto& lanes = std::get<osrm::json::Array>(lanes_iter->second);
  if (lane_index >= lanes.values.size()) {
    if (error) {
      *error = new osrmc_error{"IndexOutOfBounds", "Lane index out of bounds"};
    }
    return 0;
  }

  const auto& lane = std::get<osrm::json::Object>(lanes.values.at(lane_index));
  const auto indications_iter = lane.values.find("indications");
  if (indications_iter == lane.values.end()) {
    return 0;
  }

  const auto& indications = std::get<osrm::json::Array>(indications_iter->second);
  return static_cast<unsigned>(indications.values.size());
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return 0;
}

int
osrmc_trip_response_step_intersection_lane_valid(osrmc_trip_response_t response,
                                                 unsigned trip_index,
                                                 unsigned leg_index,
                                                 unsigned step_index,
                                                 unsigned intersection_index,
                                                 unsigned lane_index,
                                                 osrmc_error_t* error) try {
  if (!osrmc_validate_trip_response(response, error)) {
    return -1;
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);
  const auto* step = osrmc_get_trip_step(*response_typed, trip_index, leg_index, step_index, error);
  if (!step) {
    return -1;
  }

  const auto* intersection = osrmc_get_step_intersection(*step, intersection_index, error);
  if (!intersection) {
    return -1;
  }

  const auto lanes_iter = intersection->values.find("lanes");
  if (lanes_iter == intersection->values.end()) {
    if (error) {
      *error = new osrmc_error{"NoLanes", "Lanes not available for this intersection"};
    }
    return -1;
  }

  const auto& lanes = std::get<osrm::json::Array>(lanes_iter->second);
  if (lane_index >= lanes.values.size()) {
    if (error) {
      *error = new osrmc_error{"IndexOutOfBounds", "Lane index out of bounds"};
    }
    return -1;
  }

  const auto& lane = std::get<osrm::json::Object>(lanes.values.at(lane_index));
  const auto valid_iter = lane.values.find("valid");
  if (valid_iter == lane.values.end()) {
    return -1;
  }

  const auto& valid = valid_iter->second;
  if (std::holds_alternative<osrm::json::True>(valid)) {
    return 1;
  } else if (std::holds_alternative<osrm::json::False>(valid)) {
    return 0;
  } else {
    if (error) {
      *error = new osrmc_error{"InvalidValid", "Valid value is not a boolean"};
    }
    return -1;
  }
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return -1;
}

unsigned
osrmc_trip_response_step_intersection_classes_count(osrmc_trip_response_t response,
                                                    unsigned trip_index,
                                                    unsigned leg_index,
                                                    unsigned step_index,
                                                    unsigned intersection_index,
                                                    osrmc_error_t* error) try {
  if (!osrmc_validate_trip_response(response, error)) {
    return 0;
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);
  const auto* step = osrmc_get_trip_step(*response_typed, trip_index, leg_index, step_index, error);
  if (!step) {
    return 0;
  }

  const auto* intersection = osrmc_get_step_intersection(*step, intersection_index, error);
  if (!intersection) {
    return 0;
  }

  const auto classes_iter = intersection->values.find("classes");
  if (classes_iter == intersection->values.end()) {
    return 0;
  }

  const auto& classes = std::get<osrm::json::Array>(classes_iter->second);
  return static_cast<unsigned>(classes.values.size());
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return 0;
}

const char*
osrmc_trip_response_step_intersection_class(osrmc_trip_response_t response,
                                            unsigned trip_index,
                                            unsigned leg_index,
                                            unsigned step_index,
                                            unsigned intersection_index,
                                            unsigned class_index,
                                            osrmc_error_t* error) try {
  if (!osrmc_validate_trip_response(response, error)) {
    return nullptr;
  }
  auto* response_typed = reinterpret_cast<osrm::json::Object*>(response);
  const auto* step = osrmc_get_trip_step(*response_typed, trip_index, leg_index, step_index, error);
  if (!step) {
    return nullptr;
  }

  const auto* intersection = osrmc_get_step_intersection(*step, intersection_index, error);
  if (!intersection) {
    return nullptr;
  }

  const auto classes_iter = intersection->values.find("classes");
  if (classes_iter == intersection->values.end()) {
    if (error) {
      *error = new osrmc_error{"NoClasses", "Classes not available for this intersection"};
    }
    return nullptr;
  }

  const auto& classes = std::get<osrm::json::Array>(classes_iter->second);
  if (class_index >= classes.values.size()) {
    if (error) {
      *error = new osrmc_error{"IndexOutOfBounds", "Class index out of bounds"};
    }
    return nullptr;
  }

  const auto& class_str = std::get<osrm::json::String>(classes.values.at(class_index)).value;
  return class_str.c_str();
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

/* Tile */

osrmc_tile_params_t
osrmc_tile_params_construct(osrmc_error_t* error) try {
  auto* out = new osrm::TileParameters;
  out->x = 0;
  out->y = 0;
  out->z = 0;
  return reinterpret_cast<osrmc_tile_params_t>(out);
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

void
osrmc_tile_params_destruct(osrmc_tile_params_t params) {
  if (params) {
    delete reinterpret_cast<osrm::TileParameters*>(params);
  }
}

void
osrmc_tile_params_set_x(osrmc_tile_params_t params, unsigned x, osrmc_error_t* error) try {
  if (!params) {
    if (error) {
      *error = new osrmc_error{"InvalidArgument", "Params must not be null"};
    }
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::TileParameters*>(params);
  params_typed->x = x;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_tile_params_set_y(osrmc_tile_params_t params, unsigned y, osrmc_error_t* error) try {
  if (!params) {
    if (error) {
      *error = new osrmc_error{"InvalidArgument", "Params must not be null"};
    }
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::TileParameters*>(params);
  params_typed->y = y;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

void
osrmc_tile_params_set_z(osrmc_tile_params_t params, unsigned z, osrmc_error_t* error) try {
  if (!params) {
    if (error) {
      *error = new osrmc_error{"InvalidArgument", "Params must not be null"};
    }
    return;
  }
  auto* params_typed = reinterpret_cast<osrm::TileParameters*>(params);
  params_typed->z = z;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
}

osrmc_tile_response_t
osrmc_tile(osrmc_osrm_t osrm, osrmc_tile_params_t params, osrmc_error_t* error) try {
  if (!osrmc_validate_osrm(osrm, error) || !params) {
    if (error) {
      *error = new osrmc_error{"InvalidArgument", "OSRM instance and params must not be null"};
    }
    return nullptr;
  }
  auto* osrm_typed = reinterpret_cast<osrm::OSRM*>(osrm);
  auto* params_typed = reinterpret_cast<osrm::TileParameters*>(params);

  osrm::engine::api::ResultT result = std::string();
  const auto status = osrm_typed->Tile(*params_typed, result);

  if (status == osrm::Status::Ok) {
    auto* out = new std::string(std::move(std::get<std::string>(result)));
    return reinterpret_cast<osrmc_tile_response_t>(out);
  }

  if (std::holds_alternative<osrm::json::Object>(result)) {
    auto& json = std::get<osrm::json::Object>(result);
    osrmc_error_from_json(json, error);
  } else {
    if (error) {
      *error = new osrmc_error{"TileError", "Failed to generate tile"};
    }
  }

  return nullptr;
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return nullptr;
}

void
osrmc_tile_response_destruct(osrmc_tile_response_t response) {
  if (response) {
    delete reinterpret_cast<std::string*>(response);
  }
}

const char*
osrmc_tile_response_data(osrmc_tile_response_t response, size_t* size, osrmc_error_t* error) try {
  if (!osrmc_validate_tile_response(response, error)) {
    if (size) {
      *size = 0;
    }
    return nullptr;
  }
  auto* response_typed = reinterpret_cast<std::string*>(response);

  if (size) {
    *size = response_typed->size();
  }

  return response_typed->data();
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  if (size) {
    *size = 0;
  }
  return nullptr;
}

size_t
osrmc_tile_response_size(osrmc_tile_response_t response, osrmc_error_t* error) try {
  if (!osrmc_validate_tile_response(response, error)) {
    return 0;
  }
  auto* response_typed = reinterpret_cast<std::string*>(response);
  return response_typed->size();
} catch (const std::exception& e) {
  osrmc_error_from_exception(e, error);
  return 0;
}
