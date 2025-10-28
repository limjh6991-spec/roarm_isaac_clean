#!/bin/bash
# URDF Visualization Script
# Enhanced RoArm-M3 with D455 Camera

URDF_FILE="assets/roarm_m3/urdf/roarm_m3_enhanced.urdf"

echo "============================================"
echo "    RoArm-M3 Enhanced URDF Viewer"
echo "============================================"
echo ""
echo "📦 URDF File: $URDF_FILE"
echo ""

# Check if URDF file exists
if [ ! -f "$URDF_FILE" ]; then
    echo "❌ Error: URDF file not found!"
    echo "   Expected: $URDF_FILE"
    exit 1
fi

# Validate URDF
echo "🔍 Validating URDF..."
check_urdf "$URDF_FILE"
VALIDATION_STATUS=$?

if [ $VALIDATION_STATUS -ne 0 ]; then
    echo "❌ URDF validation failed!"
    exit 1
fi

echo ""
echo "✅ URDF validation successful!"
echo ""

# Check if urdf-viz is installed
if command -v urdf-viz &> /dev/null; then
    echo "🚀 Launching urdf-viz..."
    echo ""
    echo "💡 Controls:"
    echo "   - Mouse: Rotate camera"
    echo "   - Mouse Wheel: Zoom"
    echo "   - Arrow Keys: Move camera"
    echo "   - 'j': Toggle joint control mode"
    echo "   - 'c': Toggle collision geometry"
    echo "   - 'r': Reset view"
    echo "   - 'q': Quit"
    echo ""
    
    # Launch urdf-viz with web server (port 7777)
    echo "   Opening web browser at http://localhost:7777"
    urdf-viz "$URDF_FILE"
else
    echo "⏳ urdf-viz is not installed yet."
    echo ""
    echo "📥 Installation in progress. Please wait..."
    echo "   Command: cargo install urdf-viz"
    echo ""
    echo "📊 Alternative: Generate URDF graph"
    
    # Generate graphical representation
    GRAPH_FILE="urdf_graph.pdf"
    if command -v urdf_to_graphiz &> /dev/null; then
        echo "   Creating visual graph: $GRAPH_FILE"
        urdf_to_graphiz "$URDF_FILE"
        dot -Tpdf robot.gv -o "$GRAPH_FILE"
        echo "   ✅ Graph created: $GRAPH_FILE"
        
        # Try to open PDF
        if command -v xdg-open &> /dev/null; then
            xdg-open "$GRAPH_FILE" &
        fi
    fi
fi

echo ""
echo "============================================"
echo "📚 URDF Details:"
echo "============================================"
echo ""

# Print link structure
echo "🔗 Link Structure:"
grep -E "link name=" "$URDF_FILE" | sed 's/.*name="\([^"]*\)".*/  - \1/'

echo ""
echo "⚙️  Joint Structure:"
grep -E "joint name=.*type=" "$URDF_FILE" | sed 's/.*name="\([^"]*\)".*type="\([^"]*\)".*/  - \1 (\2)/'

echo ""
echo "📸 Camera Frames:"
grep -E "camera.*link name=" "$URDF_FILE" | sed 's/.*name="\([^"]*\)".*/  - \1/'

echo ""
echo "============================================"
