#!/bin/bash

# MechaOS ROS2 Workspace Setup Script
# This script sets up the complete MechaOS development environment

set -e  # Exit on any error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Print colored output
print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

print_header() {
    echo -e "\n${BLUE}===========================================${NC}"
    echo -e "${BLUE}$1${NC}"
    echo -e "${BLUE}===========================================${NC}\n"
}

check_directory() {
    if [[ ! -d "src" ]]; then
        print_error "This script must be run from the ros2_workspace directory"
        print_error "Expected directory structure:"
        print_error "  ros2_workspace/"
        print_error "  ├── src/"
        print_error "  └── setup.bash (this script)"
        exit 1
    fi
}

check_ros2() {
    print_info "Checking ROS2 installation..."
    
    if ! command -v ros2 &> /dev/null; then
        print_error "ROS2 not found. Please install ROS2 Humble first:"
        print_error "https://docs.ros.org/en/humble/Installation.html"
        exit 1
    fi
    
    # Source ROS2 if not already sourced
    if [[ -z "$ROS_DISTRO" ]]; then
        print_info "Sourcing ROS2 Humble..."
        source /opt/ros/humble/setup.bash
    fi
    
    print_success "ROS2 $ROS_DISTRO found and sourced"
}

check_python_deps() {
    print_info "Checking Python dependencies..."
    
    local missing_deps=()
    
    # Check for required Python packages
    python3 -c "import web3" 2>/dev/null || missing_deps+=("web3")
    python3 -c "import eth_account" 2>/dev/null || missing_deps+=("eth-account")
    python3 -c "import ipfshttpclient" 2>/dev/null || missing_deps+=("ipfshttpclient")
    python3 -c "import yaml" 2>/dev/null || missing_deps+=("pyyaml")
    python3 -c "import websockets" 2>/dev/null || missing_deps+=("websockets")
    
    if [[ ${#missing_deps[@]} -gt 0 ]]; then
        print_warning "Missing Python dependencies: ${missing_deps[*]}"
        read -p "Install missing dependencies? (y/n): " -n 1 -r
        echo
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            print_info "Installing Python dependencies..."
            pip3 install "${missing_deps[@]}"
            print_success "Python dependencies installed"
        else
            print_warning "Some functionality may not work without these dependencies"
        fi
    else
        print_success "All Python dependencies are available"
    fi
}

check_system_deps() {
    print_info "Checking system dependencies..."
    
    local missing_deps=()
    
    # Check for required system packages
    command -v colcon &> /dev/null || missing_deps+=("python3-colcon-common-extensions")
    command -v rosdep &> /dev/null || missing_deps+=("python3-rosdep2")
    
    if [[ ${#missing_deps[@]} -gt 0 ]]; then
        print_warning "Missing system dependencies: ${missing_deps[*]}"
        read -p "Install missing dependencies? (y/n): " -n 1 -r
        echo
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            print_info "Installing system dependencies..."
            sudo apt update
            sudo apt install -y "${missing_deps[@]}"
            print_success "System dependencies installed"
        else
            print_error "Required dependencies are missing. Cannot continue."
            exit 1
        fi
    else
        print_success "All system dependencies are available"
    fi
}

init_rosdep() {
    print_info "Checking rosdep initialization..."
    
    if [[ ! -d "/etc/ros/rosdep/sources.list.d" ]] || [[ ! -f "/etc/ros/rosdep/sources.list.d/20-default.list" ]]; then
        print_info "Initializing rosdep..."
        sudo rosdep init || print_warning "rosdep already initialized"
    fi
    
    print_info "Updating rosdep..."
    rosdep update
    print_success "rosdep is ready"
}

install_dependencies() {
    print_info "Installing ROS2 package dependencies..."
    
    # Install dependencies for all packages in the workspace
    rosdep install --from-paths src --ignore-src -r -y
    
    print_success "Package dependencies installed"
}

build_workspace() {
    print_info "Building MechaOS workspace..."
    
    # Clean previous build if it exists
    if [[ -d "build" ]] || [[ -d "install" ]] || [[ -d "log" ]]; then
        read -p "Clean previous build? (y/n): " -n 1 -r
        echo
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            print_info "Cleaning previous build..."
            rm -rf build install log
        fi
    fi
    
    # Build with colcon
    print_info "Running colcon build..."
    colcon build \
        --cmake-args -DCMAKE_BUILD_TYPE=Release \
        --symlink-install \
        --event-handlers console_direct+
    
    if [[ $? -eq 0 ]]; then
        print_success "Workspace built successfully"
    else
        print_error "Build failed"
        exit 1
    fi
}

create_env_script() {
    print_info "Creating environment setup script..."
    
    cat > env_setup.bash << 'EOF'
#!/bin/bash
# MechaOS Environment Setup
# Source this file in every terminal: source env_setup.bash

# Source ROS2
source /opt/ros/humble/setup.bash

# Source this workspace
if [[ -f "install/setup.bash" ]]; then
    source install/setup.bash
else
    echo "Warning: Workspace not built yet. Run './setup.bash' first."
fi

# Set MechaOS environment variables
export MECHAOS_WS=$(pwd)
export PYTHONPATH="$MECHAOS_WS/src:$PYTHONPATH"

# Ethereum environment (edit these values)
export ETHEREUM_RPC_URL="http://localhost:8545"
export ETHEREUM_NETWORK_ID="1337"
export CONTRACT_ADDRESS=""

# IPFS environment
export IPFS_API_URL="http://localhost:5001"
export IPFS_GATEWAY_URL="http://localhost:8080"

# ROS2 environment
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedx_cpp

echo "🤖 MechaOS environment loaded!"
echo "   Workspace: $MECHAOS_WS"
echo "   ROS2 Distro: $ROS_DISTRO"
echo "   Domain ID: $ROS_DOMAIN_ID"
EOF
    
    chmod +x env_setup.bash
    print_success "Environment setup script created: env_setup.bash"
}

create_aliases() {
    print_info "Creating useful aliases..."
    
    cat > mechaos_aliases.bash << 'EOF'
#!/bin/bash
# MechaOS Useful Aliases and Functions

# Build aliases
alias mb='colcon build'
alias mbc='colcon build --cmake-clean-cache'
alias mbs='colcon build --symlink-install'
alias mbp='colcon build --packages-select'

# Run aliases  
alias mrun='ros2 run'
alias mlaunch='ros2 launch'
alias mtopic='ros2 topic'
alias mservice='ros2 service'
alias mnode='ros2 node'

# MechaOS specific
alias bridge='ros2 run mechaos_core bridge_node'
alias robot='ros2 run robot_client task_executor_node'
alias monitor='ros2 topic echo /mechaos/robot_status'

# Quick functions
mechaos_clean() {
    echo "Cleaning MechaOS workspace..."
    rm -rf build install log
    echo "Clean complete!"
}

mechaos_build() {
    echo "Building MechaOS workspace..."
    colcon build --symlink-install
}

mechaos_test() {
    echo "Running MechaOS tests..."
    colcon test
    colcon test-result --verbose
}

mechaos_status() {
    echo "=== MechaOS Workspace Status ==="
    echo "Workspace: $(pwd)"
    echo "ROS2 Distro: $ROS_DISTRO"
    echo "Domain ID: $ROS_DOMAIN_ID"
    echo "Built packages:"
    if [[ -d "install" ]]; then
        ls install/ | grep -v "setup\|local_setup"
    else
        echo "  No packages built yet"
    fi
}

echo "🚀 MechaOS aliases loaded!"
echo "   Try: mechaos_status, mechaos_build, mechaos_clean"
EOF
    
    chmod +x mechaos_aliases.bash
    print_success "Aliases created: mechaos_aliases.bash"
}

# Create launch scripts
create_launch_scripts() {
    print_info "Creating launch scripts..."
    
    # Bridge launch script
    cat > launch_bridge.bash << 'EOF'
#!/bin/bash
# Launch MechaOS Bridge
source env_setup.bash
source mechaos_aliases.bash

echo "🌉 Starting MechaOS Bridge..."
ros2 launch mechaos_core mechaos_bridge.launch.py
EOF
    
    # Robot client launch script
    cat > launch_robot.bash << 'EOF'
#!/bin/bash
# Launch MechaOS Robot Client
source env_setup.bash
source mechaos_aliases.bash

if [[ -z "$1" ]]; then
    echo "Usage: ./launch_robot.bash <robot_config.yaml>"
    echo "Example: ./launch_robot.bash config/robot_001.yaml"
    exit 1
fi

echo "🤖 Starting MechaOS Robot Client with config: $1"
ros2 launch robot_client robot_client.launch.py config:=$1
EOF
    
    chmod +x launch_bridge.bash launch_robot.bash
    print_success "Launch scripts created: launch_bridge.bash, launch_robot.bash"
}

# Create development tools
create_dev_tools() {
    print_info "Creating development tools..."
    
    # Quick test script
    cat > test_setup.bash << 'EOF'
#!/bin/bash
# Test MechaOS Setup
source env_setup.bash

echo "=== Testing MechaOS Setup ==="

echo "✓ Checking ROS2..."
ros2 --version

echo "✓ Checking topics..."
timeout 2s ros2 topic list || echo "No topics yet (this is normal)"

echo "✓ Checking nodes..."
ros2 node list

echo "✓ Checking packages..."
if [[ -d "install" ]]; then
    echo "Built packages:"
    ls install/ | grep -v setup
else
    echo "No packages built yet"
fi

echo "✓ Testing Python imports..."
python3 -c "
try:
    import rclpy
    import web3
    print('✅ All Python imports successful')
except ImportError as e:
    print(f'❌ Import error: {e}')
"

echo "=== Setup Test Complete ==="
EOF
    
    chmod +x test_setup.bash
    print_success "Development tools created: test_setup.bash"
}

# Main setup function
main() {
    print_header "🤖 MechaOS ROS2 Workspace Setup"
    
    print_info "Starting MechaOS setup process..."
    print_info "This will set up your complete development environment"
    
    # Run all setup steps
    check_directory
    check_ros2
    check_system_deps
    init_rosdep
    check_python_deps
    install_dependencies
    build_workspace
    create_env_script
    create_aliases
    create_launch_scripts
    create_dev_tools
    
    print_header "🎉 Setup Complete!"
    
    echo -e "${GREEN}MechaOS workspace is ready!${NC}\n"
    echo -e "${YELLOW}Next steps:${NC}"
    echo -e "  1. ${BLUE}source env_setup.bash${NC} - Load the environment"
    echo -e "  2. ${BLUE}source mechaos_aliases.bash${NC} - Load useful aliases"
    echo -e "  3. ${BLUE}./test_setup.bash${NC} - Test the setup"
    echo -e "  4. ${BLUE}./launch_bridge.bash${NC} - Start the bridge"
    echo -e "  5. ${BLUE}./launch_robot.bash config/robot.yaml${NC} - Start a robot\n"
    
    echo -e "${YELLOW}Useful commands:${NC}"
    echo -e "  ${BLUE}mechaos_status${NC} - Check workspace status"
    echo -e "  ${BLUE}mechaos_build${NC} - Rebuild workspace"
    echo -e "  ${BLUE}mechaos_clean${NC} - Clean build files\n"
    print_success "Happy coding with MechaOS! 🤖🚀"
}

main