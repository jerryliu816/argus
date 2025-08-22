// Keyboard controls for robot - same mappings as teleop.py

class KeyboardController {
    constructor() {
        // Movement bindings - same as teleop.py
        this.moveBindings = {
            'i': [1, 0, 0, 0],    // forward
            'o': [1, 0, 0, -1],   // forward + right turn
            'j': [0, 0, 0, 1],    // left turn
            'l': [0, 0, 0, -1],   // right turn
            'u': [1, 0, 0, 1],    // forward + left turn
            ',': [-1, 0, 0, 0],   // backward
            '.': [-1, 0, 0, 1],   // backward + left turn
            'm': [-1, 0, 0, -1],  // backward + right turn
            'k': [0, 0, 0, 0],    // stop
            
            // Holonomic mode (with shift)
            'I': [1, 0, 0, 0],    // forward
            'O': [1, -1, 0, 0],   // forward + strafe right
            'J': [0, 1, 0, 0],    // strafe left
            'L': [0, -1, 0, 0],   // strafe right
            'U': [1, 1, 0, 0],    // forward + strafe left
            '<': [-1, 0, 0, 0],   // backward (shift + comma)
            '>': [-1, -1, 0, 0],  // backward + strafe right (shift + period)
            'M': [-1, 1, 0, 0],   // backward + strafe left
            
            // Vertical movement
            't': [0, 0, 1, 0],    // up
            'b': [0, 0, -1, 0],   // down
        };

        // Speed bindings - same as teleop.py
        this.speedBindings = {
            'q': [1.1, 1.1],     // increase all speeds
            'z': [0.9, 0.9],     // decrease all speeds
            'w': [1.1, 1],       // increase linear only
            'x': [0.9, 1],       // decrease linear only
            'e': [1, 1.1],       // increase angular only
            'c': [1, 0.9],       // decrease angular only
        };

        // Dock bindings - same as teleop.py
        this.dockBindings = {
            'd': 'dock',
            's': 'undock'
        };

        // Current movement state
        this.currentMovement = [0, 0, 0, 0];
        this.isMoving = false;
        this.keysPressed = new Set();
        this.activeMovementKey = null;
        this.repeatInterval = null;
        
        // Help visibility
        this.helpVisible = false;
        
        this.initialize();
    }

    initialize() {
        // Bind keyboard events
        document.addEventListener('keydown', this.handleKeyDown.bind(this));
        document.addEventListener('keyup', this.handleKeyUp.bind(this));
        
        // Prevent context menu on right-click
        document.addEventListener('contextmenu', (e) => e.preventDefault());
        
        // Focus on window to ensure keyboard events work
        window.focus();
        
        console.log('Keyboard controller initialized');
    }

    handleKeyDown(event) {
        // Prevent default browser behavior
        event.preventDefault();
        
        let key = event.key;
        console.log('KeyDown:', key, 'Already pressed:', this.keysPressed.has(key));
        
        // Handle special keys
        if (key === ',') key = ',';
        else if (key === '.') key = '.';
        else if (key === '<') key = '<'; // Shift + comma
        else if (key === '>') key = '>'; // Shift + period
        
        // Prevent all key repeats - we'll handle movement repeats manually
        if (this.keysPressed.has(key)) {
            console.log('Key repeat ignored:', key);
            return;
        }
        
        this.keysPressed.add(key);
        console.log('Keys pressed:', Array.from(this.keysPressed));
        
        // Handle different key types
        if (key in this.moveBindings) {
            console.log('Movement key detected:', key);
            this.handleMovementKey(key);
        } else if (key in this.speedBindings) {
            this.handleSpeedKey(key);
        } else if (key in this.dockBindings) {
            this.handleDockKey(key);
        } else if (key === 'h') {
            this.toggleHelp();
        } else {
            // Any other key stops movement
            console.log('Stopping movement for key:', key);
            this.stopMovement();
        }
    }

    handleKeyUp(event) {
        let key = event.key;
        console.log('KeyUp:', key);
        
        // Handle special keys
        if (key === ',') key = ',';
        else if (key === '.') key = '.';
        else if (key === '<') key = '<';
        else if (key === '>') key = '>';
        
        this.keysPressed.delete(key);
        console.log('Keys after release:', Array.from(this.keysPressed));
        
        // Stop movement when releasing movement keys
        if (key in this.moveBindings && key === this.activeMovementKey) {
            console.log('Stopping movement for released key:', key);
            this.stopMovement();
        }
    }

    handleMovementKey(key) {
        const movement = this.moveBindings[key];
        this.currentMovement = [...movement];
        this.isMoving = true;
        this.activeMovementKey = key;
        
        // Send initial movement command
        this.sendMovementCommand();
        
        // Start repeat interval (like terminal key repeat)
        this.startKeyRepeat();
        
        // Update UI
        this.updateMovementDisplay(key);
    }

    startKeyRepeat() {
        // Clear any existing interval
        if (this.repeatInterval) {
            clearInterval(this.repeatInterval);
        }
        
        console.log('Starting key repeat interval');
        
        // Start repeating every 100ms while key is held
        this.repeatInterval = setInterval(() => {
            console.log('Interval tick - isMoving:', this.isMoving, 'activeKey:', this.activeMovementKey);
            if (this.isMoving && this.activeMovementKey) {
                console.log('Sending repeat movement command');
                this.sendMovementCommand();
            }
        }, 100);
    }

    handleSpeedKey(key) {
        const speedChange = this.speedBindings[key];
        
        // Send speed adjustment command
        if (window.robotControls) {
            window.robotControls.adjustSpeed(speedChange[0], speedChange[1]);
        }
        
        console.log(`Speed adjustment: ${key} -> linear: ${speedChange[0]}, angular: ${speedChange[1]}`);
    }

    handleDockKey(key) {
        const command = this.dockBindings[key];
        
        // Send dock/undock command
        if (window.robotControls) {
            if (command === 'dock') {
                window.robotControls.dock();
            } else if (command === 'undock') {
                window.robotControls.undock();
            }
        }
        
        console.log(`Dock command: ${command}`);
    }

    stopMovement() {
        this.currentMovement = [0, 0, 0, 0];
        this.isMoving = false;
        this.activeMovementKey = null;
        
        // Stop repeat interval
        if (this.repeatInterval) {
            clearInterval(this.repeatInterval);
            this.repeatInterval = null;
        }
        
        // Send stop command
        this.sendMovementCommand();
        
        // Update UI
        this.updateMovementDisplay('stop');
    }

    sendMovementCommand() {
        if (window.robotControls) {
            const [x, y, z, th] = this.currentMovement;
            window.robotControls.sendMovement(x, y, z, 0, 0, th);
        }
    }

    updateMovementDisplay(key) {
        const movementDisplay = document.getElementById('current-movement');
        const lastCommandDisplay = document.getElementById('last-command');
        
        if (movementDisplay) {
            if (key === 'stop' || key === 'k') {
                movementDisplay.textContent = 'Stopped';
                movementDisplay.style.color = '#666';
            } else {
                movementDisplay.textContent = `Moving: ${key}`;
                movementDisplay.style.color = '#2196F3';
            }
        }
        
        if (lastCommandDisplay) {
            lastCommandDisplay.textContent = key === 'stop' ? 'Stop' : `Key: ${key}`;
        }
    }

    toggleHelp() {
        const helpSection = document.getElementById('keyboard-controls');
        if (helpSection) {
            this.helpVisible = !this.helpVisible;
            if (this.helpVisible) {
                helpSection.classList.remove('hidden');
            } else {
                helpSection.classList.add('hidden');
            }
        }
    }

    showHelp() {
        const helpSection = document.getElementById('keyboard-controls');
        if (helpSection) {
            helpSection.classList.remove('hidden');
            this.helpVisible = true;
        }
    }

    hideHelp() {
        const helpSection = document.getElementById('keyboard-controls');
        if (helpSection) {
            helpSection.classList.add('hidden');
            this.helpVisible = false;
        }
    }

    // Method to check if keyboard is being used
    isActive() {
        return this.keysPressed.size > 0 || this.isMoving;
    }

    // Get help text for display
    getHelpText() {
        return `
        Reading from the keyboard and Publishing to Twist!
        ---------------------------
        Moving around:
         u i o
         j k l
         m , .

        For Holonomic mode (strafing), hold down the shift key:
        ---------------------------
         U I O
         J K L
         M < >

        t : up (+z)
        b : down (-z)

        Dock/Undock controls:
        ---------------------------
        d : dock robot
        s : undock robot

        Speed controls:
        ---------------------------
        q/z : increase/decrease max speeds by 10%
        w/x : increase/decrease only linear speed by 10%
        e/c : increase/decrease only angular speed by 10%

        anything else : stop

        h : show/hide this help message
        `;
    }
}

// Export for use in other modules
window.KeyboardController = KeyboardController;