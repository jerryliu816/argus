// Camera functionality for dashboard

class CameraController {
    constructor() {
        this.thumbnailImg = null;
        this.fullImg = null;
        this.modal = null;
        this.refreshInterval = null;
        this.isRefreshing = false;
        
        // Camera status
        this.isConnected = false;
        this.lastUpdate = null;
        
        // Auto-refresh settings
        this.autoRefreshEnabled = true;
        this.refreshIntervalMs = 3000; // 3 seconds
        
        this.initialize();
    }

    initialize() {
        // Get DOM elements
        this.thumbnailImg = document.getElementById('camera-thumbnail');
        this.fullImg = document.getElementById('camera-full');
        this.modal = document.getElementById('camera-modal');
        
        if (!this.thumbnailImg || !this.fullImg || !this.modal) {
            console.error('Camera elements not found');
            return;
        }
        
        this.bindEvents();
        this.startAutoRefresh();
        
        console.log('Camera controller initialized');
    }

    bindEvents() {
        // Thumbnail click to open modal
        this.thumbnailImg.addEventListener('click', () => {
            this.openModal();
        });
        
        // Refresh buttons
        const refreshBtn = document.getElementById('camera-refresh');
        const modalRefreshBtn = document.getElementById('modal-refresh');
        
        if (refreshBtn) {
            refreshBtn.addEventListener('click', () => {
                this.refreshThumbnail();
            });
        }
        
        if (modalRefreshBtn) {
            modalRefreshBtn.addEventListener('click', () => {
                this.refreshFullImage();
            });
        }
        
        // Fullscreen button
        const fullscreenBtn = document.getElementById('camera-fullscreen');
        if (fullscreenBtn) {
            fullscreenBtn.addEventListener('click', () => {
                this.openModal();
            });
        }
        
        // Modal close button
        const closeBtn = document.getElementById('modal-close');
        if (closeBtn) {
            closeBtn.addEventListener('click', () => {
                this.closeModal();
            });
        }
        
        // Download button
        const downloadBtn = document.getElementById('modal-download');
        if (downloadBtn) {
            downloadBtn.addEventListener('click', () => {
                this.downloadImage();
            });
        }
        
        // Modal background click to close
        this.modal.addEventListener('click', (event) => {
            if (event.target === this.modal) {
                this.closeModal();
            }
        });
        
        // Keyboard shortcuts
        document.addEventListener('keydown', (event) => {
            if (event.key === 'Escape' && !this.modal.classList.contains('hidden')) {
                this.closeModal();
            }
        });
    }

    async refreshThumbnail() {
        if (this.isRefreshing) return;
        
        this.isRefreshing = true;
        this.showLoading(true);
        this.hideError();
        
        try {
            const response = await fetch('/api/camera/thumbnail?' + new Date().getTime());
            
            if (response.ok) {
                const blob = await response.blob();
                const imageUrl = URL.createObjectURL(blob);
                
                // Update thumbnail
                if (this.thumbnailImg.src.startsWith('blob:')) {
                    URL.revokeObjectURL(this.thumbnailImg.src);
                }
                
                this.thumbnailImg.src = imageUrl;
                this.isConnected = true;
                this.lastUpdate = new Date();
                
                console.log('Thumbnail updated successfully');
                
            } else {
                throw new Error(`HTTP ${response.status}: ${response.statusText}`);
            }
            
        } catch (error) {
            console.error('Failed to refresh thumbnail:', error);
            this.showError('Failed to load camera image');
            this.isConnected = false;
            
        } finally {
            this.isRefreshing = false;
            this.showLoading(false);
        }
    }

    async refreshFullImage() {
        this.showModalLoading(true);
        
        try {
            const response = await fetch('/api/camera/full?' + new Date().getTime());
            
            if (response.ok) {
                const blob = await response.blob();
                const imageUrl = URL.createObjectURL(blob);
                
                // Update full image
                if (this.fullImg.src.startsWith('blob:')) {
                    URL.revokeObjectURL(this.fullImg.src);
                }
                
                this.fullImg.src = imageUrl;
                console.log('Full image updated successfully');
                
            } else {
                throw new Error(`HTTP ${response.status}: ${response.statusText}`);
            }
            
        } catch (error) {
            console.error('Failed to refresh full image:', error);
            alert('Failed to load full resolution image');
            
        } finally {
            this.showModalLoading(false);
        }
    }

    openModal() {
        this.modal.classList.remove('hidden');
        document.body.style.overflow = 'hidden'; // Prevent background scrolling
        
        // Load full resolution image
        this.refreshFullImage();
    }

    closeModal() {
        this.modal.classList.add('hidden');
        document.body.style.overflow = 'auto'; // Restore scrolling
        
        // Clean up blob URL to prevent memory leaks
        if (this.fullImg.src.startsWith('blob:')) {
            URL.revokeObjectURL(this.fullImg.src);
            this.fullImg.src = '';
        }
    }

    downloadImage() {
        if (!this.fullImg.src) {
            alert('No image to download');
            return;
        }
        
        // Create download link
        const link = document.createElement('a');
        link.href = this.fullImg.src;
        link.download = `robot_camera_${new Date().toISOString().replace(/[:.]/g, '-')}.jpg`;
        document.body.appendChild(link);
        link.click();
        document.body.removeChild(link);
        
        console.log('Image download triggered');
    }

    showLoading(show) {
        const loadingElement = document.getElementById('camera-loading');
        if (loadingElement) {
            if (show) {
                loadingElement.classList.remove('hidden');
            } else {
                loadingElement.classList.add('hidden');
            }
        }
    }

    showModalLoading(show) {
        // Simple loading indicator for modal
        const refreshBtn = document.getElementById('modal-refresh');
        if (refreshBtn) {
            if (show) {
                refreshBtn.textContent = '⏳ Loading...';
                refreshBtn.disabled = true;
            } else {
                refreshBtn.textContent = '🔄 Refresh';
                refreshBtn.disabled = false;
            }
        }
    }

    showError(message) {
        const errorElement = document.getElementById('camera-error');
        if (errorElement) {
            errorElement.textContent = message;
            errorElement.classList.remove('hidden');
        }
    }

    hideError() {
        const errorElement = document.getElementById('camera-error');
        if (errorElement) {
            errorElement.classList.add('hidden');
        }
    }

    startAutoRefresh() {
        if (this.refreshInterval) {
            clearInterval(this.refreshInterval);
        }
        
        // Initial refresh
        setTimeout(() => this.refreshThumbnail(), 1000);
        
        // Set up auto-refresh
        this.refreshInterval = setInterval(() => {
            if (this.autoRefreshEnabled && !this.isRefreshing) {
                this.refreshThumbnail();
            }
        }, this.refreshIntervalMs);
        
        console.log(`Auto-refresh started with ${this.refreshIntervalMs}ms interval`);
    }

    stopAutoRefresh() {
        if (this.refreshInterval) {
            clearInterval(this.refreshInterval);
            this.refreshInterval = null;
        }
        console.log('Auto-refresh stopped');
    }

    setAutoRefresh(enabled, intervalMs = null) {
        this.autoRefreshEnabled = enabled;
        
        if (intervalMs) {
            this.refreshIntervalMs = intervalMs;
        }
        
        if (enabled) {
            this.startAutoRefresh();
        } else {
            this.stopAutoRefresh();
        }
    }

    // Get camera status
    getStatus() {
        return {
            connected: this.isConnected,
            lastUpdate: this.lastUpdate,
            autoRefresh: this.autoRefreshEnabled,
            refreshInterval: this.refreshIntervalMs
        };
    }

    // Cleanup method
    cleanup() {
        this.stopAutoRefresh();
        
        // Clean up blob URLs
        if (this.thumbnailImg && this.thumbnailImg.src.startsWith('blob:')) {
            URL.revokeObjectURL(this.thumbnailImg.src);
        }
        
        if (this.fullImg && this.fullImg.src.startsWith('blob:')) {
            URL.revokeObjectURL(this.fullImg.src);
        }
        
        console.log('Camera controller cleaned up');
    }

    // Manual capture for AI analysis (future feature)
    async captureForAnalysis() {
        try {
            const response = await fetch('/api/camera/full?' + new Date().getTime());
            if (response.ok) {
                const blob = await response.blob();
                return blob;
            } else {
                throw new Error(`Failed to capture: ${response.statusText}`);
            }
        } catch (error) {
            console.error('Failed to capture for analysis:', error);
            throw error;
        }
    }
}

// Handle page unload cleanup
window.addEventListener('beforeunload', () => {
    if (window.cameraController) {
        window.cameraController.cleanup();
    }
});

// Export for use in other modules
window.CameraController = CameraController;