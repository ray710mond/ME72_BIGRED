#
# Extra Shell Initialization
#

# Ignore Ctrl-D.  Otherwise Ctrl-D would log you out. 
set -o ignoreeof

# Personal/safety stuff.  Use trash-put instead of permanent rm, etc.
alias mv='mv -i'
alias rm='trash-put'
alias dir='ls -alv'


# Make the current directory the active workspace and source it
# --- Persistent ROS2 workspace support ---
ROS2_WS_FILE="$HOME/.ros2_active_ws"

# Load persistent workspace if present
if [ -z "$ROS2_WS" ] && [ -f "$ROS2_WS_FILE" ]; then
    export ROS2_WS="$(cat "$ROS2_WS_FILE")"
    if [ -f "$ROS2_WS/install/setup.bash" ]; then
        source "$ROS2_WS/install/setup.bash"
        echo "🟢 Loaded active workspace: $ROS2_WS"
    fi
fi

# Set active workspace and persist it
rosset() {
    export ROS2_WS="$(pwd)"
    echo "$ROS2_WS" > "$ROS2_WS_FILE"

    if [ -f "$ROS2_WS/install/setup.bash" ]; then
        source "$ROS2_WS/install/setup.bash"
        echo "🟢 Active workspace set to: $ROS2_WS"
        echo "💾 Persisted for future terminals."
    else
        echo "⚠️  $ROS2_WS has no install/setup.bash yet — run rosmkv to build first."
        echo "💾 Workspace path still persisted."
    fi
}

# Source the active workspace (from anywhere)
rosso() {
    if [ -z "$ROS2_WS" ]; then
        echo "❌ No active workspace set. Run rosset in your workspace first."
        return 1
    fi
    if [ -f "$ROS2_WS/install/setup.bash" ]; then
        source "$ROS2_WS/install/setup.bash"
        echo "🔁 Sourced active workspace: $ROS2_WS"
    else
        echo "⚠️  Active workspace ($ROS2_WS) has no install/setup.bash."
    fi
}

# Build (make) the active ROS2 workspace and re-source it
rosmkv() {
    if [ -z "$ROS2_WS" ]; then
        echo "❌ No active workspace set. Run rosset in your workspace first."
        return 1
    fi

    WS="$ROS2_WS"
    cd "$WS" || { echo "❌ Could not cd to $WS"; return 1; }

    echo "🔧 Building workspace at: $WS"
    START=$(date +%s)

    colcon build --symlink-install \
        --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -Wall -Wextra -Wpedantic || return $?

    DURATION=$(( $(date +%s) - START ))

    if [ -f "$WS/install/setup.bash" ]; then
        source "$WS/install/setup.bash"
        echo "✅ Build complete and sourced: $WS/install/setup.bash (⏱ ${DURATION}s)"
    else
        echo "⚠️  Build finished, but no install/setup.bash found. (⏱ ${DURATION}s)"
    fi
}

# Clean build artifacts (safe way)
rosclean() {
    WS="${ROS2_WS:-$(pwd)}"
    cd "$WS" || return
    echo "🧹 Cleaning build/, install/, and log/ in $WS"
    rm -rf build install log
}

rosshow() {
    echo "-----------------------------------------"
    echo "🔍 ROS2 Workspace Status"
    echo "-----------------------------------------"

    if [ -z "$ROS2_WS" ]; then
        echo "❌ No active workspace set."
        echo "   Run:  rosset"
        return 1
    fi

    echo "📁 Active workspace: $ROS2_WS"

    if [ ! -d "$ROS2_WS" ]; then
        echo "❌ Directory does NOT exist!"
        return 1
    fi

    # Check if built
    if [ -f "$ROS2_WS/install/setup.bash" ]; then
        echo "🟢 Built: install/setup.bash present"
    else
        echo "🟡 Not built yet (missing install/setup.bash)"
        echo "   Run:  rosmkv"
    fi

    # Check if workspace looks like a ROS2 workspace
    if [ -d "$ROS2_WS/src" ]; then
        echo "📦 Packages in src/: $(ls -1 "$ROS2_WS/src" | wc -l)"
    else
        echo "⚠️  No src/ directory found — unusual for a ROS2 workspace"
    fi

    # Optional: show active environment info
    echo "🌐 Current ROS_DOMAIN_ID: ${ROS_DOMAIN_ID:-default}"
    echo "-----------------------------------------"
}


# Check whether we have any personal executable files under ~/bin.
if [ -d ~/bin ]; then
    PATH="~/bin:$PATH"
   # echo "Added personal executables under ~/bin"
fi

# Check whether we are using ROS.
if [ -f ~/.bash_ros2 ]; then
    source ~/.bash_ros2
elif [ -f ~/.bash_ros ]; then
    source ~/.bash_ros
else
    echo "Welcome - ready to drive a robot!"
fi
