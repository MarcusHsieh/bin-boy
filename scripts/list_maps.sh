#!/bin/bash
# List all saved maps with metadata

MAP_DIR=~/bin-boy/maps

echo "========================================="
echo "  Available Maps"
echo "========================================="

if [ ! -d "$MAP_DIR" ]; then
    echo "No maps directory found."
    echo "Create maps with: bash ~/bin-boy/scripts/save_map.sh"
    exit 0
fi

cd $MAP_DIR
COUNT=0

for yaml_file in *.yaml; do
    if [ -f "$yaml_file" ]; then
        MAP_NAME="${yaml_file%.yaml}"
        PGM_FILE="$MAP_NAME.pgm"
        META_FILE="$MAP_NAME.metadata.txt"

        echo ""
        echo "Map: $MAP_NAME"

        if [ -f "$META_FILE" ]; then
            cat "$META_FILE" | sed 's/^/  /'
        else
            if [ -f "$PGM_FILE" ]; then
                # Get size from PGM header (line 2 contains width height)
                SIZE=$(sed -n '2p' "$PGM_FILE" 2>/dev/null | awk '{print $1 "x" $2 " pixels"}' 2>/dev/null || echo "unknown")
                echo "  Size: $SIZE"
            fi
            echo "  Files: $yaml_file, $PGM_FILE"
        fi

        COUNT=$((COUNT + 1))
    fi
done

echo ""
echo "========================================="
echo "Total maps: $COUNT"

if [ $COUNT -eq 0 ]; then
    echo ""
    echo "No maps found. Create one with:"
    echo "  bash ~/bin-boy/scripts/save_map.sh my_map_name"
fi
