//
//  load_settings_from_file.cpp
//
//  Created by Jonathan Tompson on 7/31/12.
//

#if defined(WIN32) || defined(_WIN32)
#include <windows.h>
#include <tchar.h> 
#include <stdio.h>
#include <strsafe.h>
#pragma comment(lib, "User32.lib")
#endif
#include <fstream>
#include <string>
#include <stdexcept>
#include <iostream>
#include "load_settings_from_file.h"
#include "file_io/csv_handle_read.h"
#include "data_str/vector_managed.h"
#include "string_util/string_util.h"

using std::wstring;
using std::string;
using std::runtime_error;
using file_io::CSVHandleRead;
using data_str::VectorManaged;
using std::cout;
using std::endl;

void parseToken(ProgramSettings* settings, VectorManaged<char*>* cur_token, std::string filename);

void loadSettingsFromFile(ProgramSettings* settings, std::string filename) {
  CSVHandleRead reader(filename);

  // Default settings (so that program isn't in some weird state)
  settings->load_forest_from_file = false;
  settings->num_trees = 4; 
  settings->num_workers = 2;
  settings->tree_height = 20;
  settings->min_info_gain = 1000.0f;
  settings->max_pixels_per_image_per_label = 1000;
  settings->num_wl_samples_per_node = 1000;
  settings->max_num_images = 20000;
  settings->num_bootstrap_passes = 0;
  settings->bootstrap_tree_height = 20;
  settings->wl_func_type = 0;

  VectorManaged<char*> cur_token;  // Each element is a csv in the line
  static const bool inc_whitespace = false;

  // Keep reading tokens until we're at the end
  while (!reader.checkEOF()) {
    reader.readNextToken(& cur_token, inc_whitespace);  // Get the next elem
    if (cur_token.size() > 0)
      parseToken(settings, &cur_token, filename);  // process object

    cur_token.clear();  // Clear the token
  }
  reader.close();

  cout << "Loaded settings from file '" << filename << "': " << endl;
  cout << "    load_forest_from_file = " << settings->load_forest_from_file << endl;
  cout << "    num_trees = " << settings->num_trees << endl;
  cout << "    num_workers = " << settings->num_workers << endl;
  cout << "    tree_height = " << settings->tree_height << endl;
  cout << "    min_info_gain = " << settings->min_info_gain << endl;
  cout << "    max_pixels_per_image_per_label = " << settings->max_pixels_per_image_per_label << endl;
  cout << "    num_wl_samples_per_node = " << settings->num_wl_samples_per_node << endl;
  cout << "    max_num_images = " << settings->max_num_images << endl;
  cout << "    num_bootstrap_passes = " << settings->num_bootstrap_passes << endl;
  cout << "    bootstrap_tree_height = " << settings->bootstrap_tree_height << endl;
  cout << "    wl_func_type = " << settings->wl_func_type << endl << endl;
}

void parseToken(ProgramSettings* settings, VectorManaged<char*>* cur_token, std::string filename) {
  std::string setting_name(*cur_token->at(0));

  // Switch statement here would be ideal, but using wchar_t * in switch is 
  // messy (must be const case values)
  if (setting_name == "//") {
    // Do nothing for comment blocks
  } else if (setting_name == "") {
    // Do nothing for empty lines
  } else {
    std::string value(*cur_token->at(1));
    if (setting_name == "load_forest_from_file") {
      if (string_util::Str2Num<int>(value) == 1) {
        settings->load_forest_from_file = true;
      } else {
        settings->load_forest_from_file = false;
      }
    } else if (setting_name == "num_trees") {
      settings->num_trees = static_cast<uint32_t>(string_util::Str2Num<int>(value));
    } else if (setting_name == "num_workers") {
      settings->num_workers = static_cast<uint32_t>(string_util::Str2Num<int>(value));
    } else if (setting_name == "tree_height") {
      settings->tree_height = static_cast<uint32_t>(string_util::Str2Num<int>(value));
    } else if (setting_name == "min_info_gain") {
      settings->min_info_gain = static_cast<float>(string_util::Str2Num<float>(value));
    } else if (setting_name == "max_pixels_per_image_per_label") {
      settings->max_pixels_per_image_per_label = static_cast<uint32_t>(string_util::Str2Num<int>(value));
    } else if (setting_name == "num_wl_samples_per_node") {
      settings->num_wl_samples_per_node = static_cast<uint32_t>(string_util::Str2Num<int>(value));
    } else if (setting_name == "max_num_images") {
      settings->max_num_images = static_cast<uint32_t>(string_util::Str2Num<int>(value));
    } else if (setting_name == "num_bootstrap_passes") {
      settings->num_bootstrap_passes = static_cast<uint32_t>(string_util::Str2Num<int>(value));
    } else if (setting_name == "bootstrap_tree_height") {
      settings->bootstrap_tree_height = static_cast<uint32_t>(string_util::Str2Num<int>(value));
    } else if (setting_name == "wl_func_type") {
      settings->wl_func_type = static_cast<uint32_t>(string_util::Str2Num<int>(value));
    } else {
      throw std::runtime_error(string("ERROR: Unrecognized setting name in file:" +
                                      filename));
    }
  } 
}