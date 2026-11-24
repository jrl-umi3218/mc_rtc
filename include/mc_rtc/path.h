/*
 * Copyright 2015-2024 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rtc/utils_api.h>

#include <filesystem>
#include <string>

namespace fs = std::filesystem;

namespace mc_rtc
{

/** Returns the path to the OS temporary directory
 *
 * \param suffix Added (with a path separator) to the returned path
 */
MC_RTC_UTILS_DLLAPI std::string temp_directory_path(const std::string & suffix = "");

/** Returns the path to the user's config directory
 *
 * On Linux/macOS this returns ${HOME}/.config/mc_rtc folder
 *
 * On Windows this returns the %APPDATA%/mc_rtc folder
 *
 * \param suffix Added (with a path separator) to the returned path
 */
MC_RTC_UTILS_DLLAPI std::string user_config_directory_path(const std::string & suffix = "");

/** Constructs a platform-specific path to the local share directory for mc_rtc.
 *
 * This function returns a string representing the full path to the local share directory for `mc_rtc`,
 * appending the provided `suffix` to the base directory. The base directory varies by platform:
 * - On Windows: Uses the `%APPDATA%/mc_rtc` directory.
 * - On Unix-like systems: Uses the `$HOME/.local/share/mc_rtc` directory.
 *
 * \param suffix The subdirectory or filename to append to the mc_rtc local share path.
 * \return A string representing the full path to the local share directory.
 */
MC_RTC_UTILS_DLLAPI std::string local_share_directory(const std::string & suffix = "");

/** Extracts the filename from a given filesystem path.
 *
 * This function returns the filename part of the specified `path`. If `keepExtension` is true,
 * the returned string includes the file extension. If false, only the filename stem (i.e., without extension) is
 * returned.
 *
 * \param path The input filesystem path.
 * \param keepExtension If true, keep the file extension in the result; otherwise, exclude it.
 * \return The filename as a std::string, with or without extension depending on `keepExtension`.
 */
MC_RTC_UTILS_DLLAPI std::string filename(const fs::path & path, bool keepExtension = false);

/** Convert a URI to a filesystem path.
 *
 * This function handles URIs with different schemes and converts them to a
 * `std::filesystem::path` object. It currently supports:
 * - `package://` URIs: Typically used in ROS or simulation environments.
 * - `file://` URIs: Absolute file paths.
 * - Plain strings: Treated as direct filesystem paths.
 *
 * For `package://` URIs, the function attempts to resolve the package path
 * using available ROS methods:
 *
 * \param uri The URI to be converted. Can be a `package://`, `file://`, or direct file path.
 * \param default_dir Default directory used to resolve unknown package names when ROS support is not available.
 *
 * \return std::filesystem::path A resolved path corresponding to the given URI.
 */
MC_RTC_UTILS_DLLAPI fs::path convertURI(const std::string & uri, [[maybe_unused]] std::string_view default_dir = "");

/**
 * @brief Creates a unique temporary directory path with the given prefix.
 *
 * This function generates a unique path in the system's temporary directory,
 * using the specified prefix and a random pattern. It creates the directory
 * before returning the path as a string.
 *
 * @param prefix Prefix to use for the temporary directory name.
 * @return std::string The full path to the created temporary directory.
 */
MC_RTC_UTILS_DLLAPI std::string make_temporary_path(const std::string & prefix);

/**
 * @brief Generates a unique path string by replacing placeholder '%' characters with random hex digits.
 * * This function models the behavior of `boost::filesystem::unique_path`. It scans the input
 * pattern and replaces every occurrence of the percent sign (`%`) with a cryptographically
 * pseudo-random hexadecimal character `[0-9a-f]`.
 * * @param pattern The model string containing `%` placeholders (e.g., "temp_dir/file_%%%%.tmp").
 * * @return A new `std::string` with all placeholders substituted with random hex characters.
 * * @threadsafe Yes. Utilizes a `thread_local` pseudo-random number generator (`std::mt19937`),
 * ensuring thread isolation without the concurrency overhead of a mutex lock.
 */
MC_RTC_UTILS_DLLAPI std::string unique_path(std::string pattern);

} // namespace mc_rtc
