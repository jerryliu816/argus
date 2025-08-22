// Main controls module - coordinates all dashboard functionality

class RobotControls {
    constructor() {
        this.websocket = null;
        this.keyboard = null;
        this.touch = null;
        this.camera = null;
        
        // Current state
        this.speedSettings = { linear: 0.5, angular: 1.0 };
        this.currentTwist = { linear: { x: 0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 } };
        this.robotStatus = { connected: false, docked: false };
        this.cameraStatus = { connected: false };
        
        // Input device detection
        this.inputDevice = 'unknown';
        this.lastInputTime = 0;
        
        this.initialize();
    }

    initialize() {
        console.log('RobotControls initialize() - FIXED VERSION');
        // Initialize all controllers
        this.websocket = new WebSocketController();
        this.keyboard = { 
            stopMovement: () => {}, 
            isActive: () => false,
            showHelp: () => {},
            hideHelp: () => {}
        }; // Dummy keyboard object
        this.touch = new TouchController();
        this.camera = new CameraController();
        
        // Make available globally
        window.robotControls = this;
        window.websocketController = this.websocket;
        window.keyboardController = this.keyboard;
        window.touchController = this.touch;
        window.cameraController = this.camera;
        
        // Detect input device
        this.detectInputDevice();
        
        // Bind UI events
        this.bindUIEvents();
        
        // Start status updates
        this.startStatusUpdates();
        
        console.log('Robot controls initialized');
    }

    detectInputDevice() {
        // Always use touch mode only
        this.setInputDevice('touch');
    }

    setupInputDetection() {
        // No input detection - always stay in touch mode
    }

    setInputDevice(device) {
        // Prevent infinite loops
        if (this.inputDevice === device) {
            return;
        }
        
        this.inputDevice = device;
        
        const deviceIndicator = document.getElementById('device-type');
        const touchControls = document.getElementById('touch-controls');
        
        // Force touch mode only
        console.log('Setting touch mode - touchControls element:', touchControls);
        if (deviceIndicator) deviceIndicator.textContent = '📱 Touch Mode';
        if (touchControls) {
            touchControls.classList.remove('hidden');
            console.log('Touch controls shown, classes:', touchControls.className);
        }
        
        console.log(`Input device set to: ${device}`);
    }

    bindUIEvents() {
        // Emergency stop button
        const emergencyBtn = document.getElementById('emergency-stop');
        if (emergencyBtn) {
            emergencyBtn.addEventListener('click', () => {
                this.emergencyStop();
            });
        }
        
        // Speed sliders
        const linearSlider = document.getElementById('linear-speed');
        const angularSlider = document.getElementById('angular-speed');
        
        if (linearSlider) {
            linearSlider.addEventListener('input', (e) => {
                this.speedSettings.linear = parseFloat(e.target.value);
                this.updateSpeedDisplay();
                this.sendSpeedUpdate();
            });
        }
        
        if (angularSlider) {
            angularSlider.addEventListener('input', (e) => {
                this.speedSettings.angular = parseFloat(e.target.value);
                this.updateSpeedDisplay();
                this.sendSpeedUpdate();
            });
        }
        
        // Dock/Undock buttons
        const dockBtn = document.getElementById('dock-btn');
        const undockBtn = document.getElementById('undock-btn');
        
        if (dockBtn) {
            dockBtn.addEventListener('click', () => {
                this.dock();
            });
        }
        
        if (undockBtn) {
            undockBtn.addEventListener('click', () => {
                this.undock();
            });
        }
    }

    // Movement control methods
    sendMovement(linearX, linearY, linearZ, angularX, angularY, angularZ) {
        // Apply speed scaling
        const scaledLinearX = linearX * this.speedSettings.linear;
        const scaledLinearY = linearY * this.speedSettings.linear;
        const scaledLinearZ = linearZ * this.speedSettings.linear;
        const scaledAngularZ = angularZ * this.speedSettings.angular;
        
        // Update current twist
        this.currentTwist = {
            linear: { x: scaledLinearX, y: scaledLinearY, z: scaledLinearZ },
            angular: { x: angularX, y: angularY, z: scaledAngularZ }
        };
        
        // Send via WebSocket
        const success = this.websocket.sendMovement(
            scaledLinearX, scaledLinearY, scaledLinearZ,
            angularX, angularY, scaledAngularZ
        );
        
        // Update UI
        this.updateCurrentSpeedDisplay();
        
        return success;
    }

    adjustSpeed(linearMultiplier, angularMultiplier) {
        // Adjust speed settings
        if (linearMultiplier !== 1) {
            this.speedSettings.linear = Math.max(0.1, Math.min(2.0, 
                this.speedSettings.linear * linearMultiplier));
        }
        
        if (angularMultiplier !== 1) {
            this.speedSettings.angular = Math.max(0.1, Math.min(3.0, 
                this.speedSettings.angular * angularMultiplier));
        }
        
        // Update UI
        this.updateSpeedDisplay();
        this.sendSpeedUpdate();
        
        console.log(`Speed adjusted - Linear: ${this.speedSettings.linear.toFixed(1)}, Angular: ${this.speedSettings.angular.toFixed(1)}`);
    }

    sendSpeedUpdate() {
        // Update sliders to match current settings
        const linearSlider = document.getElementById('linear-speed');
        const angularSlider = document.getElementById('angular-speed');
        
        if (linearSlider) linearSlider.value = this.speedSettings.linear;
        if (angularSlider) angularSlider.value = this.speedSettings.angular;
    }

    dock() {
        const success = this.websocket.sendDockCommand();
        if (success) {
            console.log('Dock command sent');
        } else {
            console.error('Failed to send dock command');
        }
        return success;
    }

    undock() {
        const success = this.websocket.sendUndockCommand();
        if (success) {
            console.log('Undock command sent');
        } else {
            console.error('Failed to send undock command');
        }
        return success;
    }

    emergencyStop() {
        console.log('EMERGENCY STOP ACTIVATED');
        
        // Stop all movement
        this.websocket.sendStopCommand();
        
        // Stop controllers
        if (this.touch) {
            this.touch.emergencyStop();
        }
        
        
        // Update UI
        const movementDisplay = document.getElementById('current-movement');
        const lastCommandDisplay = document.getElementById('last-command');
        
        if (movementDisplay) {
            movementDisplay.textContent = '🛑 EMERGENCY STOP';
            movementDisplay.style.color = '#f44336';
        }
        
        if (lastCommandDisplay) {
            lastCommandDisplay.textContent = 'Emergency Stop';
        }
        
        // Flash the emergency button
        const emergencyBtn = document.getElementById('emergency-stop');
        if (emergencyBtn) {
            emergencyBtn.style.animation = 'none';
            setTimeout(() => {
                emergencyBtn.style.animation = 'pulse 0.5s ease-in-out 3';
            }, 10);
        }
    }

    // Status update methods
    startStatusUpdates() {
        // Update system status every 2 seconds
        setInterval(() => {
            this.updateSystemStatus();
        }, 2000);
    }

    async updateSystemStatus() {
        try {
            const response = await fetch('/api/status');
            if (response.ok) {
                const status = await response.json();
                this.updateStatus(status);
            }
        } catch (error) {
            console.error('Failed to fetch system status:', error);
        }
    }

    updateStatus(status) {
        // Update robot status
        this.robotStatus = {
            connected: status.robot_connected || false,
            docked: status.docked || false
        };
        
        // Update camera status
        this.cameraStatus = {
            connected: status.camera_connected || false
        };
        
        // Update battery status
        this.batteryPercentage = status.battery_percentage;
        
        // Update UI
        this.updateStatusDisplay();
        
        // Update speed settings if provided
        if (status.current_speed) {
            this.speedSettings = { ...status.current_speed };
            this.updateSpeedDisplay();
        }
    }

    updateStatusDisplay() {
        // Robot status
        const robotStatusElement = document.getElementById('robot-status');
        if (robotStatusElement) {
            if (this.robotStatus.connected) {
                robotStatusElement.textContent = 'Connected';
                robotStatusElement.className = 'status-value connected';
            } else {
                robotStatusElement.textContent = 'Disconnected';
                robotStatusElement.className = 'status-value disconnected';
            }
        }
        
        // Camera status
        const cameraStatusElement = document.getElementById('camera-status');
        if (cameraStatusElement) {
            if (this.cameraStatus.connected) {
                cameraStatusElement.textContent = 'Connected';
                cameraStatusElement.className = 'status-value connected';
            } else {
                cameraStatusElement.textContent = 'Disconnected';
                cameraStatusElement.className = 'status-value disconnected';
            }
        }
        
        // Battery status
        const batteryStatusElement = document.getElementById('battery-status');
        if (batteryStatusElement) {
            if (this.batteryPercentage !== null && this.batteryPercentage !== undefined) {
                batteryStatusElement.textContent = `${this.batteryPercentage}%`;
                // Set color based on battery level
                if (this.batteryPercentage > 50) {
                    batteryStatusElement.className = 'status-value connected'; // Green
                } else if (this.batteryPercentage > 20) {
                    batteryStatusElement.className = 'status-value unknown'; // Yellow
                } else {
                    batteryStatusElement.className = 'status-value disconnected'; // Red
                }
            } else {
                batteryStatusElement.textContent = 'Unknown';
                batteryStatusElement.className = 'status-value unknown';
            }
        }
    }

    updateSpeedDisplay() {
        const linearSpeedValue = document.getElementById('linear-speed-value');
        const angularSpeedValue = document.getElementById('angular-speed-value');
        
        if (linearSpeedValue) {
            linearSpeedValue.textContent = `${this.speedSettings.linear.toFixed(1)} m/s`;
        }
        
        if (angularSpeedValue) {
            angularSpeedValue.textContent = `${this.speedSettings.angular.toFixed(1)} rad/s`;
        }
    }

    updateCurrentSpeedDisplay() {
        const currentLinear = document.getElementById('current-linear');
        const currentAngular = document.getElementById('current-angular');
        
        if (currentLinear) {
            const linearMag = Math.sqrt(
                this.currentTwist.linear.x ** 2 + 
                this.currentTwist.linear.y ** 2
            );
            currentLinear.textContent = `${linearMag.toFixed(2)} m/s`;
        }
        
        if (currentAngular) {
            currentAngular.textContent = `${Math.abs(this.currentTwist.angular.z).toFixed(2)} rad/s`;
        }
    }

    // Get overall system status
    getSystemStatus() {
        return {
            websocket: this.websocket.getStats(),
            robot: this.robotStatus,
            camera: this.cameraStatus,
            speeds: this.speedSettings,
            currentTwist: this.currentTwist,
            inputDevice: this.inputDevice,
            controllers: {
                keyboard: this.keyboard.isActive(),
                touch: this.touch.isActive()
            }
        };
    }
}

// Initialize when DOM is loaded
document.addEventListener('DOMContentLoaded', () => {
    console.log('Initializing Robot Control Dashboard...');
    new RobotControls();
});

// Add CSS animation for emergency button
const style = document.createElement('style');
style.textContent = `
@keyframes pulse {
    0% { transform: scale(1); }
    50% { transform: scale(1.05); }
    100% { transform: scale(1); }
}
`;
document.head.appendChild(style);