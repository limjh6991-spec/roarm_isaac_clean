#!/bin/bash
# Checkpoint Cleanup Script
# Keep only checkpoints at specified intervals to save disk space

CHECKPOINT_DIR="logs/rl_training_curriculum/checkpoints"
KEEP_INTERVAL=5000  # Keep every 5K steps
DRY_RUN=false

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --interval)
            KEEP_INTERVAL="$2"
            shift 2
            ;;
        --dry-run)
            DRY_RUN=true
            shift
            ;;
        *)
            echo "Unknown option: $1"
            echo "Usage: $0 [--interval STEPS] [--dry-run]"
            exit 1
            ;;
    esac
done

echo "🧹 Checkpoint Cleanup"
echo "===================="
echo "Directory: $CHECKPOINT_DIR"
echo "Keep interval: every ${KEEP_INTERVAL} steps"
echo "Dry run: $DRY_RUN"
echo ""

if [ ! -d "$CHECKPOINT_DIR" ]; then
    echo "❌ Error: Checkpoint directory not found!"
    exit 1
fi

cd "$CHECKPOINT_DIR" || exit 1

# Find latest checkpoint first
latest_steps=0
for file in *_steps.zip; do
    [ -f "$file" ] || continue
    steps=$(echo "$file" | grep -oP '\d+(?=_steps\.zip)')
    if [ "$steps" -gt "$latest_steps" ]; then
        latest_steps=$steps
    fi
done

echo "📊 Latest checkpoint: ${latest_steps} steps"
echo ""

# Process checkpoints
total=0
kept=0
removed=0
space_saved=0

for file in *_steps.zip; do
    [ -f "$file" ] || continue
    
    # Extract step number
    steps=$(echo "$file" | grep -oP '\d+(?=_steps\.zip)')
    
    total=$((total + 1))
    
    # Keep if: multiple of interval OR latest
    if [ $((steps % KEEP_INTERVAL)) -eq 0 ] || [ "$steps" -eq "$latest_steps" ]; then
        kept=$((kept + 1))
        echo "✓ Keep: $file ($steps steps)"
    else
        removed=$((removed + 1))
        file_size=$(stat -f%z "$file" 2>/dev/null || stat -c%s "$file" 2>/dev/null || echo "182000")
        space_saved=$((space_saved + file_size))
        
        if [ "$DRY_RUN" = false ]; then
            rm -f "$file"
            rm -f "${file/_steps.zip/_vecnormalize.pkl}" 2>/dev/null
            rm -f "${file/_steps.zip/_steps.pkl}" 2>/dev/null
            echo "✗ Removed: $file"
        else
            echo "✗ Would remove: $file"
        fi
    fi
done

# Convert bytes to human readable
space_saved_mb=$((space_saved / 1024 / 1024))

echo ""
echo "📊 Summary:"
echo "  Total checkpoints: $total"
echo "  Kept: $kept"
echo "  Removed: $removed"
echo "  Space saved: ${space_saved_mb}MB"

if [ "$DRY_RUN" = true ]; then
    echo ""
    echo "⚠️  This was a DRY RUN - no files were actually deleted"
    echo "   To perform the cleanup, run without --dry-run"
fi

echo ""
echo "✅ Checkpoint cleanup complete!"
