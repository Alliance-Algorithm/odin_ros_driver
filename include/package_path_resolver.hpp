#pragma once

#include <array>
#include <cctype>
#include <cstdlib>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <system_error>

#ifdef __linux__
#include <unistd.h>
#endif

namespace odin_ros_driver::paths {

namespace detail {

inline std::filesystem::path normalize_path(const std::filesystem::path& path) {
    std::error_code ec;
    const auto absolute = path.is_absolute() ? path : std::filesystem::absolute(path, ec);
    const auto normalized_input = ec ? path.lexically_normal() : absolute;

    ec.clear();
    const auto canonical = std::filesystem::weakly_canonical(normalized_input, ec);
    return ec ? normalized_input.lexically_normal() : canonical;
}

inline bool path_is_directory(const std::filesystem::path& path) {
    std::error_code ec;
    return std::filesystem::exists(path, ec) && std::filesystem::is_directory(path, ec);
}

inline bool path_is_file(const std::filesystem::path& path) {
    std::error_code ec;
    return std::filesystem::exists(path, ec) && std::filesystem::is_regular_file(path, ec);
}

inline std::string package_env_prefix(const std::string& package_name) {
    std::string key;
    key.reserve(package_name.size());
    for (unsigned char c : package_name) {
        key.push_back(std::isalnum(c) ? static_cast<char>(std::toupper(c)) : '_');
    }
    return key;
}

inline std::filesystem::path executable_path() {
#ifdef __linux__
    std::array<char, 4096> buffer{};
    const auto len = ::readlink("/proc/self/exe", buffer.data(), buffer.size() - 1);
    if (len > 0) {
        buffer[static_cast<std::size_t>(len)] = '\0';
        return normalize_path(buffer.data());
    }
#endif
    std::error_code ec;
    return normalize_path(std::filesystem::current_path(ec));
}

inline std::filesystem::path installed_package_dir(const std::filesystem::path& base,
                                                   const std::string& package_name) {
    const auto candidate = base / "share" / package_name;
    if (path_is_directory(candidate) && path_is_directory(candidate / "config")) {
        return candidate;
    }
    return {};
}

inline std::filesystem::path source_package_dir(const std::filesystem::path& base,
                                                const std::string& package_name) {
    if (base.filename() == package_name &&
        path_is_file(base / "package.xml") &&
        path_is_directory(base / "config")) {
        return base;
    }
    return {};
}

inline std::filesystem::path workspace_source_package_dir(const std::filesystem::path& base,
                                                          const std::string& package_name) {
    const auto candidate = base / "src" / package_name;
    if (path_is_file(candidate / "package.xml") && path_is_directory(candidate / "config")) {
        return candidate;
    }
    return {};
}

inline std::filesystem::path search_from(const std::filesystem::path& start,
                                         const std::string& package_name) {
    if (start.empty()) {
        return {};
    }

    auto current = normalize_path(start);
    if (path_is_file(current)) {
        current = current.parent_path();
    }

    while (!current.empty()) {
        if (const auto installed = installed_package_dir(current, package_name); !installed.empty()) {
            return installed;
        }
        if (const auto source = source_package_dir(current, package_name); !source.empty()) {
            return source;
        }
        if (const auto workspace = workspace_source_package_dir(current, package_name); !workspace.empty()) {
            return workspace;
        }

        if (current == current.root_path()) {
            break;
        }
        current = current.parent_path();
    }

    return {};
}

}  // namespace detail

inline std::filesystem::path find_package_directory(const std::string& package_name) {
    const auto env_prefix = detail::package_env_prefix(package_name);

    if (const char* package_dir = std::getenv((env_prefix + "_PACKAGE_DIR").c_str());
        package_dir && *package_dir) {
        const auto candidate = detail::normalize_path(package_dir);
        if (detail::path_is_directory(candidate / "config")) {
            return candidate;
        }
    }

    if (const char* share_dir = std::getenv((env_prefix + "_SHARE_DIR").c_str());
        share_dir && *share_dir) {
        const auto candidate = detail::normalize_path(share_dir);
        if (detail::path_is_directory(candidate / "config")) {
            return candidate;
        }
    }

    if (const auto from_executable = detail::search_from(detail::executable_path(), package_name);
        !from_executable.empty()) {
        return from_executable;
    }

    std::error_code ec;
    if (const auto from_cwd = detail::search_from(std::filesystem::current_path(ec), package_name);
        !from_cwd.empty()) {
        return from_cwd;
    }

    throw std::runtime_error(
        "Failed to locate package directory for '" + package_name +
        "' relative to the executable or current working directory.");
}

inline std::filesystem::path find_config_dir(const std::string& package_name) {
    const auto env_prefix = detail::package_env_prefix(package_name);
    if (const char* config_dir = std::getenv((env_prefix + "_CONFIG_DIR").c_str());
        config_dir && *config_dir) {
        const auto candidate = detail::normalize_path(config_dir);
        if (detail::path_is_directory(candidate)) {
            return candidate;
        }
    }

    const auto package_dir = find_package_directory(package_name);
    const auto config_dir = package_dir / "config";
    if (!detail::path_is_directory(config_dir)) {
        throw std::runtime_error("Config directory not found for package '" + package_name + "'.");
    }
    return config_dir;
}

inline std::filesystem::path resolve_path(const std::string& package_name,
                                          const std::string& configured_path,
                                          const std::filesystem::path& default_relative_path) {
    const auto package_dir = find_package_directory(package_name);
    const auto config_dir = package_dir / "config";

    if (configured_path.empty()) {
        return detail::normalize_path(package_dir / default_relative_path);
    }

    const std::filesystem::path requested_path(configured_path);
    if (requested_path.is_absolute()) {
        return detail::normalize_path(requested_path);
    }

    std::error_code ec;
    const auto cwd_candidate = std::filesystem::current_path(ec) / requested_path;
    if (!ec && std::filesystem::exists(cwd_candidate, ec)) {
        return detail::normalize_path(cwd_candidate);
    }

    const auto package_candidate = package_dir / requested_path;
    ec.clear();
    if (std::filesystem::exists(package_candidate, ec)) {
        return detail::normalize_path(package_candidate);
    }

    const auto config_candidate = config_dir / requested_path;
    ec.clear();
    if (std::filesystem::exists(config_candidate, ec)) {
        return detail::normalize_path(config_candidate);
    }

    return detail::normalize_path(package_candidate);
}

}  // namespace odin_ros_driver::paths
