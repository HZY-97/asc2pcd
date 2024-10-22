#!/bin/bash

# Function to check if a directory should be excluded
should_exclude() {
  case $1 in
    build|install|thirdParty|multicast)
      return 0
      ;;
    *)
      return 1
      ;;
  esac
}

# Function to check if a file should be excluded
should_exclude_file() {
  case $1 in
    json.hpp|json_fwd.hpp)
      return 0
      ;;
    *)
      return 1
      ;;
  esac
}

# Function to format a single file using clang-format
format_file() {
  clang-format -i "$1"
  echo "Formatted: $1"
}

# Function to recursively process files
process_files() {
  for file in "$1"/*; do
    if [ -d "$file" ]; then
      # Directory
      if should_exclude "$(basename "$file")"; then
        continue
      fi
      process_files "$file"
    elif [ -f "$file" ]; then
      # File
      if should_exclude_file "$(basename "$file")"; then
        echo "Excluded: $file"
        continue
      fi
      ext="${file##*.}"
      case $ext in
        h|cpp|hpp|cc|c)
          format_file "$file" &
          ;;
      esac
    fi
  done
  wait
}

# Start processing from the current directory
process_files .

echo "Formatting completed."
