#!/bin/bash

# API URL for the specified GitHub directory
API_URL="https://api.github.com/repos/Pedro-Pathing/Quickstart/contents/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedro?ref=master"

echo "Fetching file list from GitHub..."

# Use curl and jq to fetch the file list and download each file directly into the current directory
curl -s "$API_URL" | jq -r '.[] | select(.type == "file") | .download_url' | while read -r url; do
    if [ -n "$url" ]; then
        filename=$(basename "$url")
        echo "Downloading $filename..."
        curl -s -L -O "$url"
    fi
done

echo "Download complete!"
