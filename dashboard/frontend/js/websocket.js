// WebSocket communication for real-time robot control

class WebSocketController {
    constructor() {
        this.ws = null;
        this.isConnected = false;
        this.reconnectAttempts = 0;
        this.maxReconnectAttempts = 5;
        this.reconnectDelay = 1000; // Start with 1 second
        this.maxReconnectDelay = 30000; // Max 30 seconds
        this.reconnectTimer = null;
        
        // Message queue for when disconnected
        this.messageQueue = [];
        this.maxQueueSize = 100;
        
        // Stats
        this.messagesSent = 0;
        this.messagesReceived = 0;
        this.lastPingTime = null;
        this.latency = null;
        
        this.initialize();
    }

    initialize() {
        this.connect();
        
        // Periodic connection check
        setInterval(() => {
            this.checkConnection();
        }, 5000);
        
        console.log('WebSocket controller initialized');
    }

    connect() {
        try {
            // Determine WebSocket URL
            const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
            const host = window.location.host;
            const wsUrl = `${protocol}//${host}/ws/control`;
            
            console.log(`Connecting to WebSocket: ${wsUrl}`);
            
            this.ws = new WebSocket(wsUrl);
            
            this.ws.onopen = this.handleOpen.bind(this);
            this.ws.onmessage = this.handleMessage.bind(this);
            this.ws.onclose = this.handleClose.bind(this);
            this.ws.onerror = this.handleError.bind(this);
            
        } catch (error) {
            console.error('Failed to create WebSocket connection:', error);
            this.scheduleReconnect();
        }
    }

    handleOpen(event) {
        console.log('WebSocket connected successfully');
        this.isConnected = true;
        this.reconnectAttempts = 0;
        this.reconnectDelay = 1000;
        
        // Update UI
        this.updateConnectionStatus('connected');
        
        // Process queued messages
        this.processMessageQueue();
        
        // Send ping to measure latency
        this.sendPing();
    }

    handleMessage(event) {
        this.messagesReceived++;
        
        try {
            const message = JSON.parse(event.data);
            
            switch (message.type) {
                case 'ack':
                    // Command acknowledgment
                    this.handleAck(message);
                    break;
                    
                case 'pong':
                    // Ping response
                    this.handlePong(message);
                    break;
                    
                case 'status':
                    // Status update
                    this.handleStatusUpdate(message);
                    break;
                    
                default:
                    console.log('Unknown message type:', message.type);
            }
            
        } catch (error) {
            console.error('Failed to parse WebSocket message:', error);
        }
    }

    handleClose(event) {
        console.log('WebSocket connection closed:', event.code, event.reason);
        this.isConnected = false;
        
        // Update UI
        this.updateConnectionStatus('disconnected');
        
        // Schedule reconnection if not a normal closure
        if (event.code !== 1000) {
            this.scheduleReconnect();
        }
    }

    handleError(event) {
        console.error('WebSocket error:', event);
        this.isConnected = false;
        this.updateConnectionStatus('disconnected');
    }

    handleAck(message) {
        // Command acknowledged
        console.log('Command acknowledged:', message.command);
    }

    handlePong(message) {
        if (this.lastPingTime) {
            this.latency = Date.now() - this.lastPingTime;
            console.log(`Latency: ${this.latency}ms`);
        }
    }

    handleStatusUpdate(message) {
        // Update UI with status information
        if (window.robotControls) {
            window.robotControls.updateStatus(message.data);
        }
    }

    send(message) {
        if (this.isConnected && this.ws.readyState === WebSocket.OPEN) {
            try {
                this.ws.send(JSON.stringify(message));
                this.messagesSent++;
                return true;
            } catch (error) {
                console.error('Failed to send message:', error);
                this.queueMessage(message);
                return false;
            }
        } else {
            // Queue message for later
            this.queueMessage(message);
            return false;
        }
    }

    queueMessage(message) {
        if (this.messageQueue.length >= this.maxQueueSize) {
            this.messageQueue.shift(); // Remove oldest message
        }
        this.messageQueue.push(message);
    }

    processMessageQueue() {
        while (this.messageQueue.length > 0 && this.isConnected) {
            const message = this.messageQueue.shift();
            this.send(message);
        }
    }

    scheduleReconnect() {
        if (this.reconnectTimer) {
            clearTimeout(this.reconnectTimer);
        }
        
        if (this.reconnectAttempts >= this.maxReconnectAttempts) {
            console.error('Max reconnection attempts reached');
            this.updateConnectionStatus('failed');
            return;
        }
        
        this.reconnectAttempts++;
        console.log(`Scheduling reconnection attempt ${this.reconnectAttempts} in ${this.reconnectDelay}ms`);
        
        this.reconnectTimer = setTimeout(() => {
            this.connect();
        }, this.reconnectDelay);
        
        // Exponential backoff
        this.reconnectDelay = Math.min(this.reconnectDelay * 2, this.maxReconnectDelay);
    }

    checkConnection() {
        if (this.isConnected && this.ws.readyState === WebSocket.OPEN) {
            this.sendPing();
        } else if (!this.isConnected && this.reconnectAttempts < this.maxReconnectAttempts) {
            console.log('Connection lost, attempting to reconnect...');
            this.scheduleReconnect();
        }
    }

    sendPing() {
        this.lastPingTime = Date.now();
        this.send({
            type: 'ping',
            timestamp: this.lastPingTime
        });
    }

    updateConnectionStatus(status) {
        const statusElement = document.getElementById('connection-status');
        if (statusElement) {
            statusElement.textContent = status.charAt(0).toUpperCase() + status.slice(1);
            statusElement.className = `status-value ${status}`;
        }
    }

    // Public methods for sending commands
    sendMovement(linearX, linearY, linearZ, angularX, angularY, angularZ) {
        return this.send({
            type: 'move',
            linear_x: linearX,
            linear_y: linearY,
            linear_z: linearZ,
            angular_x: angularX,
            angular_y: angularY,
            angular_z: angularZ
        });
    }

    sendSpeedAdjustment(linearMultiplier, angularMultiplier) {
        return this.send({
            type: 'speed',
            linear: linearMultiplier,
            angular: angularMultiplier
        });
    }

    sendDockCommand() {
        return this.send({
            type: 'dock'
        });
    }

    sendUndockCommand() {
        return this.send({
            type: 'undock'
        });
    }

    sendStopCommand() {
        return this.send({
            type: 'stop'
        });
    }

    // Get connection statistics
    getStats() {
        return {
            connected: this.isConnected,
            reconnectAttempts: this.reconnectAttempts,
            messagesSent: this.messagesSent,
            messagesReceived: this.messagesReceived,
            latency: this.latency,
            queuedMessages: this.messageQueue.length
        };
    }

    // Cleanup
    disconnect() {
        if (this.reconnectTimer) {
            clearTimeout(this.reconnectTimer);
            this.reconnectTimer = null;
        }
        
        if (this.ws) {
            this.ws.close(1000, 'Manual disconnect');
        }
        
        this.isConnected = false;
        this.messageQueue = [];
        
        console.log('WebSocket disconnected');
    }
}

// Export for use in other modules
window.WebSocketController = WebSocketController;