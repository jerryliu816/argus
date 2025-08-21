// Touch controls for robot - virtual joystick implementation

class TouchController {
    constructor() {
        this.joystick = null;
        this.joystickKnob = null;
        this.joystickRect = null;
        this.isDragging = false;
        this.centerX = 0;
        this.centerY = 0;
        this.maxDistance = 0;
        this.currentX = 0;
        this.currentY = 0;
        
        // Movement values
        this.linearX = 0;
        this.linearY = 0;
        this.angularZ = 0;
        
        // Update interval
        this.updateInterval = null;
        
        this.initialize();
    }

    initialize() {
        this.joystick = document.getElementById('joystick');
        this.joystickKnob = document.getElementById('joystick-knob');
        
        if (!this.joystick || !this.joystickKnob) {
            console.error('Joystick elements not found');
            return;
        }
        
        this.setupJoystick();
        this.bindEvents();
        
        console.log('Touch controller initialized');
    }

    setupJoystick() {
        this.joystickRect = this.joystick.getBoundingClientRect();
        this.centerX = this.joystickRect.width / 2;
        this.centerY = this.joystickRect.height / 2;
        this.maxDistance = (this.joystickRect.width - 60) / 2; // Account for knob size
        
        // Reset knob to center
        this.resetKnob();
    }

    bindEvents() {
        // Mouse events for desktop
        this.joystick.addEventListener('mousedown', this.handleStart.bind(this));
        document.addEventListener('mousemove', this.handleMove.bind(this));
        document.addEventListener('mouseup', this.handleEnd.bind(this));
        
        // Touch events for mobile
        this.joystick.addEventListener('touchstart', this.handleStart.bind(this));
        document.addEventListener('touchmove', this.handleMove.bind(this));
        document.addEventListener('touchend', this.handleEnd.bind(this));
        
        // Prevent context menu
        this.joystick.addEventListener('contextmenu', (e) => e.preventDefault());
        
        // Handle window resize
        window.addEventListener('resize', () => {
            setTimeout(() => this.setupJoystick(), 100);
        });
    }

    handleStart(event) {
        event.preventDefault();
        this.isDragging = true;
        
        // Update joystick rect in case of layout changes
        this.joystickRect = this.joystick.getBoundingClientRect();
        this.centerX = this.joystickRect.width / 2;
        this.centerY = this.joystickRect.height / 2;
        
        // Start update loop
        this.startUpdateLoop();
        
        // Handle initial position
        this.handleMove(event);
    }

    handleMove(event) {
        if (!this.isDragging) return;
        
        event.preventDefault();
        
        let clientX, clientY;
        
        if (event.type.includes('touch')) {
            if (event.touches.length === 0) return;
            clientX = event.touches[0].clientX;
            clientY = event.touches[0].clientY;
        } else {
            clientX = event.clientX;
            clientY = event.clientY;
        }
        
        // Calculate position relative to joystick center
        const rect = this.joystick.getBoundingClientRect();
        const deltaX = clientX - (rect.left + this.centerX);
        const deltaY = clientY - (rect.top + this.centerY);
        
        // Calculate distance from center
        const distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
        
        // Constrain to joystick bounds
        if (distance <= this.maxDistance) {
            this.currentX = deltaX;
            this.currentY = deltaY;
        } else {
            // Normalize to max distance
            const angle = Math.atan2(deltaY, deltaX);
            this.currentX = Math.cos(angle) * this.maxDistance;
            this.currentY = Math.sin(angle) * this.maxDistance;
        }
        
        // Update knob position
        this.updateKnobPosition();
        
        // Calculate movement values
        this.calculateMovement();
    }

    handleEnd(event) {
        if (!this.isDragging) return;
        
        event.preventDefault();
        this.isDragging = false;
        
        // Reset to center
        this.resetKnob();
        
        // Stop movement
        this.linearX = 0;
        this.linearY = 0;
        this.angularZ = 0;
        
        // Send stop command
        this.sendMovement();
        
        // Stop update loop
        this.stopUpdateLoop();
        
        // Update UI
        this.updateMovementDisplay('stop');
    }

    updateKnobPosition() {
        const knobX = this.centerX + this.currentX;
        const knobY = this.centerY + this.currentY;
        
        this.joystickKnob.style.left = knobX + 'px';
        this.joystickKnob.style.top = knobY + 'px';
        this.joystickKnob.style.transform = 'translate(-50%, -50%)';
    }

    resetKnob() {
        this.currentX = 0;
        this.currentY = 0;
        this.updateKnobPosition();
    }

    calculateMovement() {
        // Normalize positions to -1.0 to 1.0 range
        const normalizedX = this.currentX / this.maxDistance;
        const normalizedY = -this.currentY / this.maxDistance; // Invert Y axis
        
        // Calculate movement values
        // X axis controls forward/backward (linear.x)
        // Y axis controls rotation (angular.z)
        this.linearX = normalizedY;  // Forward/backward
        this.linearY = normalizedX;  // Strafe left/right
        this.angularZ = -normalizedX * 0.5; // Rotation (reduced sensitivity)
        
        // Deadzone to prevent jitter
        const deadzone = 0.1;
        if (Math.abs(this.linearX) < deadzone) this.linearX = 0;
        if (Math.abs(this.linearY) < deadzone) this.linearY = 0;
        if (Math.abs(this.angularZ) < deadzone) this.angularZ = 0;
    }

    sendMovement() {
        if (window.robotControls) {
            window.robotControls.sendMovement(
                this.linearX,
                this.linearY, 
                0, // linear.z
                0, // angular.x
                0, // angular.y
                this.angularZ
            );
        }
    }

    startUpdateLoop() {
        if (this.updateInterval) return;
        
        this.updateInterval = setInterval(() => {
            if (this.isDragging) {
                this.sendMovement();
                this.updateMovementDisplay('joystick');
            }
        }, 50); // 20 Hz update rate
    }

    stopUpdateLoop() {
        if (this.updateInterval) {
            clearInterval(this.updateInterval);
            this.updateInterval = null;
        }
    }

    updateMovementDisplay(source) {
        const movementDisplay = document.getElementById('current-movement');
        const lastCommandDisplay = document.getElementById('last-command');
        
        if (movementDisplay) {
            if (source === 'stop') {
                movementDisplay.textContent = 'Stopped';
                movementDisplay.style.color = '#666';
            } else {
                const linearMag = Math.sqrt(this.linearX * this.linearX + this.linearY * this.linearY);
                const angularMag = Math.abs(this.angularZ);
                
                if (linearMag > 0.05 || angularMag > 0.05) {
                    movementDisplay.textContent = 'Moving: Joystick';
                    movementDisplay.style.color = '#2196F3';
                } else {
                    movementDisplay.textContent = 'Stopped';
                    movementDisplay.style.color = '#666';
                }
            }
        }
        
        if (lastCommandDisplay) {
            if (source === 'stop') {
                lastCommandDisplay.textContent = 'Stop';
            } else {
                lastCommandDisplay.textContent = `Joystick (${this.linearX.toFixed(2)}, ${this.angularZ.toFixed(2)})`;
            }
        }
    }

    // Emergency stop
    emergencyStop() {
        this.isDragging = false;
        this.resetKnob();
        this.linearX = 0;
        this.linearY = 0;
        this.angularZ = 0;
        this.sendMovement();
        this.stopUpdateLoop();
        this.updateMovementDisplay('stop');
    }

    // Method to check if touch is being used
    isActive() {
        return this.isDragging || Math.abs(this.linearX) > 0.01 || Math.abs(this.angularZ) > 0.01;
    }

    // Get current movement status
    getStatus() {
        return {
            active: this.isActive(),
            linearX: this.linearX,
            linearY: this.linearY,
            angularZ: this.angularZ,
            knobPosition: { x: this.currentX, y: this.currentY }
        };
    }
}

// Export for use in other modules
window.TouchController = TouchController;