// Global variables
let sidebarCollapsed = false;
let mediaRecorder = null;
let audioChunks = [];
let isRecordingHand = false;    
let isRecording = false;
let recordingTimer = null;
let recordingStartTime = null;
let currentAudio = null;
let currentAudioId = null;
let audioInstances = new Map();

let userPerMissionId = 1595123198513;
let deviceStatus = false
const devices = [
  { id: 1595123198513, label: 'MOCAP_test_device_1' },
  { id: 5465198512316, label: 'MOCAP_test_device_2' },
  { id: 1951195195198, label: 'MOCAP_test_device_3' },
  { id: 1695198515615, label: 'MOCAP_test_device_4' }
];



// Get the select element from the DOM
const selectElement = document.getElementById('deviceSelect');
// Loop through the data and create an option for each item
devices.forEach(device => {
    // Create a new option element
    const option = document.createElement('option');

    // Set the value and the text of the option
    option.value = device.id;
    option.textContent = device.label;

    // Add the new option to the select bar
    selectElement.appendChild(option);
});
// DOM elements
const statusDot = document.getElementById("statusDot")
const messageInput = document.getElementById('messageInput');
const sendButton = document.getElementById('sendButton');
const recordButton = document.getElementById('recordButton');
const recordHandButton = document.getElementById('recordHandButton');
const chatMessages = document.getElementById('chatMessages');
const recordingIndicator = document.getElementById('recordingIndicator');
const recordingTimerDisplay = document.getElementById('recordingTimer');
const sidebar = document.getElementById('sidebar');
const mainContent = document.getElementById('mainContent');
const sidebarOverlay = document.getElementById('sidebarOverlay');

if(deviceStatus){
    statusDot.className = "status-dot online"
}
else{
    statusDot.className = "status-dot offlin"
}
// Initialize audio recording
async function initializeAudio() {
    try {
        const stream = await navigator.mediaDevices.getUserMedia({ audio: true });
        mediaRecorder = new MediaRecorder(stream);
        
        mediaRecorder.ondataavailable = (event) => {
            if (event.data.size > 0) {
                audioChunks.push(event.data);
                console.log(event.data);
            }
        };
        
        mediaRecorder.onstop = () => {
            const audioBlob = new Blob(audioChunks, { type: 'audio/wav' });
            audioChunks = [];
            handleAudioRecording(audioBlob);
        };
        
    } catch (error) {
        console.error('Error accessing microphone:', error);
        alert('Microphone access is required for voice recording. Please allow microphone access and refresh the page.');
    }
}

// Enhanced toggle sidebar function
function toggleSidebar() {
    const isMobile = window.innerWidth <= 768;
    
    if (isMobile) {
        // Mobile behavior - toggle from top
        const isActive = sidebar.classList.contains('active');
        
        if (isActive) {
            // Hide sidebar
            sidebar.classList.remove('active');
            sidebarOverlay.classList.remove('active');
            document.body.style.overflow = '';
        } else {
            // Show sidebar
            sidebar.classList.add('active');
            sidebarOverlay.classList.add('active');
            document.body.style.overflow = 'hidden'; // Prevent background scrolling
        }
    } else {
        // Desktop behavior - toggle from left
        sidebarCollapsed = !sidebarCollapsed;
        
        if (sidebarCollapsed) {
            sidebar.classList.add('collapsed');
            mainContent.classList.add('expanded');
        } else {
            sidebar.classList.remove('collapsed');
            mainContent.classList.remove('expanded');
        }
    }
}

// Create audio player component
function createAudioPlayer(audioBlob, messageId) {
    const audioUrl = URL.createObjectURL(audioBlob);
    
    // Generate random waveform bars for visual effect
    const waveformBars = Array.from({length: 20}, () => Math.random() * 15 + 5)
        .map(height => `<div class="waveform-bar" style="height: ${height}px;"></div>`)
        .join('');
    
    return `
        <div class="audio-message">
            <div class="audio-player">
                <button class="audio-play-btn" data-audio-id="${messageId}" onclick="toggleAudioPlayback('${audioUrl}', '${messageId}')">
                    <!-- Play Icon -->
                    <svg class="play-icon" viewBox="0 0 20 20" fill="currentColor">
                        <path fill-rule="evenodd" d="M10 18a8 8 0 100-16 8 8 0 000 16zM9.555 7.168A1 1 0 008 8v4a1 1 0 001.555.832l3-2a1 1 0 000-1.664l-3-2z"/>
                    </svg>
                    <!-- Pause Icon -->
                    <svg class="pause-icon" viewBox="0 0 20 20" fill="currentColor">
                        <path fill-rule="evenodd" d="M18 10a8 8 0 11-16 0 8 8 0 0116 0zM7 8a1 1 0 012 0v4a1 1 0 11-2 0V8zm5-1a1 1 0 00-1 1v4a1 1 0 102 0V8a1 1 0 00-1-1z"/>
                    </svg>
                    <!-- Loading Icon -->
                    <svg class="loading-icon" viewBox="0 0 20 20" fill="currentColor">
                        <path fill-rule="evenodd" d="M4 2a1 1 0 011 1v2.101a7.002 7.002 0 0111.601 2.566 1 1 0 11-1.885.666A5.002 5.002 0 005.999 7H9a1 1 0 010 2H4a1 1 0 01-1-1V3a1 1 0 011-1zm.008 9.057a1 1 0 011.276.61A5.002 5.002 0 0014.001 13H11a1 1 0 110-2h5a1 1 0 011 1v5a1 1 0 11-2 0v-2.101a7.002 7.002 0 01-11.601-2.566 1 1 0 01.61-1.276z"/>
                    </svg>
                </button>
                
                <div class="audio-info">
                    <div class="audio-progress-container">
                        <div class="audio-progress" onclick="seekAudio(event, '${messageId}')">
                            <div class="audio-progress-bar" id="progress-${messageId}"></div>
                        </div>
                        <div class="audio-time" id="time-${messageId}">0:00</div>
                    </div>
                    <div class="audio-waveform" id="waveform-${messageId}">
                        ${waveformBars}
                    </div>
                </div>
            </div>
        </div>
    `;
}

// Enhanced audio playback with proper play/pause functionality
function toggleAudioPlayback(audioUrl, messageId) {
    const button = document.querySelector(`[data-audio-id="${messageId}"]`);
    const progressBar = document.getElementById(`progress-${messageId}`);
    const timeDisplay = document.getElementById(`time-${messageId}`);
    const waveformBars = document.querySelectorAll(`#waveform-${messageId} .waveform-bar`);
    
    // If another audio is playing, stop it first
    if (currentAudio && currentAudioId !== messageId && !currentAudio.paused) {
        stopCurrentAudio();
    }
    
    // Check if this audio exists and is currently playing
    if (audioInstances.has(messageId)) {
        const audio = audioInstances.get(messageId);
        if (!audio.paused) {
            // Audio is playing, so pause it
            audio.pause();
            button.classList.remove('playing');
            currentAudio = null;
            currentAudioId = null;
            return;
        } else {
            // Audio is paused, so resume it
            audio.play().then(() => {
                button.classList.add('playing');
                currentAudio = audio;
                currentAudioId = messageId;
            }).catch(error => {
                console.error('Error resuming audio:', error);
                button.classList.remove('playing');
            });
            return;
        }
    }
    
    // Create new audio instance if doesn't exist
    const audio = new Audio(audioUrl);
    audioInstances.set(messageId, audio);
    
    // Set up event listeners for this audio instance
    audio.addEventListener('loadstart', () => {
        button.classList.add('loading');
    });
    
    audio.addEventListener('canplay', () => {
        button.classList.remove('loading');
    });
    
    audio.addEventListener('timeupdate', () => {
        if (currentAudioId === messageId) {
            const progress = (audio.currentTime / audio.duration) * 100;
            progressBar.style.width = `${progress || 0}%`;
            
            // Update time display
            const currentTime = formatTime(audio.currentTime);
            timeDisplay.textContent = currentTime;
            
            // Animate waveform bars
            const activeIndex = Math.floor((audio.currentTime / audio.duration) * waveformBars.length);
            waveformBars.forEach((bar, index) => {
                if (index <= activeIndex) {
                    bar.classList.add('active');
                } else {
                    bar.classList.remove('active');
                }
            });
        }
    });
    
    audio.addEventListener('ended', () => {
        button.classList.remove('playing');
        progressBar.style.width = '0%';
        timeDisplay.textContent = '0:00';
        waveformBars.forEach(bar => bar.classList.remove('active'));
        currentAudio = null;
        currentAudioId = null;
        // Reset audio to beginning
        audio.currentTime = 0;
    });
    
    audio.addEventListener('error', (e) => {
        console.error('Audio playback error:', e);
        button.classList.remove('loading', 'playing');
        currentAudio = null;
        currentAudioId = null;
        alert('Error playing audio. Please try again.');
    });
    
    // Start playing the audio
    button.classList.add('loading');
    audio.play().then(() => {
        button.classList.remove('loading');
        button.classList.add('playing');
        currentAudio = audio;
        currentAudioId = messageId;
    }).catch(error => {
        console.error('Error playing audio:', error);
        button.classList.remove('loading', 'playing');
        alert('Error playing audio. Please try again.');
    });
}

// Stop currently playing audio
function stopCurrentAudio() {
    if (currentAudio) {
        currentAudio.pause();
        // Find and update the button state
        if (currentAudioId) {
            const button = document.querySelector(`[data-audio-id="${currentAudioId}"]`);
            if (button) {
                button.classList.remove('playing');
            }
            
            // Reset progress bar and waveform
            const progressBar = document.getElementById(`progress-${currentAudioId}`);
            const timeDisplay = document.getElementById(`time-${currentAudioId}`);
            const waveformBars = document.querySelectorAll(`#waveform-${currentAudioId} .waveform-bar`);
            
            if (progressBar) progressBar.style.width = '0%';
            if (timeDisplay) timeDisplay.textContent = '0:00';
            if (waveformBars) waveformBars.forEach(bar => bar.classList.remove('active'));
        }
        
        currentAudio = null;
        currentAudioId = null;
    }
}

// Seek audio to specific position
function seekAudio(event, messageId) {
    const audio = audioInstances.get(messageId);
    if (!audio) return;
    
    const progressBar = event.currentTarget;
    const rect = progressBar.getBoundingClientRect();
    const percent = (event.clientX - rect.left) / rect.width;
    const newTime = percent * audio.duration;
    
    if (isFinite(newTime)) {
        audio.currentTime = newTime;
    }
}

// Format time for display
function formatTime(seconds) {
    if (!isFinite(seconds)) return '0:00';
    const mins = Math.floor(seconds / 60);
    const secs = Math.floor(seconds % 60);
    return `${mins}:${secs.toString().padStart(2, '0')}`;
}

// Add message to chat
function addMessage(text, isUser = false, isAudio = false, audioBlob = null) {
    const messageDiv = document.createElement('div');
    const messageId = 'msg-' + Date.now() + '-' + Math.random().toString(36).substr(2, 9);
    messageDiv.className = `message ${isUser ? 'user-message' : 'bot-message'}`;
    
    const currentTime = new Date().toLocaleTimeString([], {hour: '2-digit', minute:'2-digit'});
    
    let messageContent = '';
    if (isAudio && audioBlob) {
        messageContent = createAudioPlayer(audioBlob, messageId);
    } else {
        messageContent = `<div class="message-text">${text}</div>`;
    }
    
    messageDiv.innerHTML = `
        <div class="message-avatar">
            <div class="avatar-icon">${isUser ? '👤' : '🤖'}</div>
        </div>
        <div class="message-content">
            ${messageContent}
            <div class="message-time">${currentTime}</div>
        </div>
    `;
    
    chatMessages.appendChild(messageDiv);
    chatMessages.scrollTop = chatMessages.scrollHeight;
}
// // Handle audio recording
// function handleAudioRecording(audioBlob) {
//     // Add audio message to chat
//     addMessage('', true, true, audioBlob);
    
//     // Here you would typically send the audio to your FastAPI backend
//     // For demo purposes, we'll simulate a response
//     setTimeout(() => {
//         const responses = [
//             "I received your audio message. Let me process that for you.",
//             "Thanks for the voice message! I'm analyzing what you said.",
//             "I heard your audio. Here's my response based on what you recorded.",
//             "Voice message received and processed successfully!"
//         ];
//         const randomResponse = responses[Math.floor(Math.random() * responses.length)];
//         addMessage(randomResponse);
//     }, 1500);
// }


async function handleAudioRecording(audioBlob) {
    try {
        addMessage('', true, true, audioBlob);
        const arrayBuffer = await audioBlob.arrayBuffer();
        const response = await fetch("/upload-audio", {
            method: "POST",
            headers: {
                "Content-Type": "audio/wav"
            },
            body: arrayBuffer
        });

        const result = await response.json();
        console.log("Server response:", result);

        addMessage(result.info, true);

    } catch (err) {
        // Create temporary error message
        const errorMsg = document.createElement('div');
        errorMsg.className = 'message bot-message';
        errorMsg.innerHTML = `
            <div class="message-avatar">
                <div class="avatar-icon">⚠️</div>
            </div>
            <div class="message-content">
                <div class="message-text">There was an error uploading your audio. Please try again.</div>
                <div class="message-time">Just now</div>
            </div>
        `;
        chatMessages.appendChild(errorMsg);
        chatMessages.scrollTop = chatMessages.scrollHeight;

        // Remove message after 2 seconds
        setTimeout(() => {
            if (errorMsg.parentNode) {
                errorMsg.parentNode.removeChild(errorMsg);
            }
        }, 2000);
    }
}

async function callDummyBot() {
    try {
        const response = await fetch("/dummy_bot");
        const data = await response.json();
        // Call addMessage as a bot (not user)
        addMessage(data.message, false);
    } catch (err) {
        console.error("API call failed", err);
    }
}

// Start recording
function startRecording() {
    if (!mediaRecorder) {
        alert('Microphone not initialized. Please refresh the page and allow microphone access.');
        return;
    }
    
    // Stop any currently playing audio
    stopCurrentAudio();
    
    isRecording = true;
    recordButton.className = 'record-button recording';
    recordingIndicator.classList.add('active');
    recordingStartTime = Date.now();
    
    // Start recording timer
    recordingTimer = setInterval(updateRecordingTimer, 100);
    
    mediaRecorder.start();
}

// Stop recording
function stopRecording() {
    if (!isRecording) return;
    
    isRecording = false;
    recordButton.className = 'record-button processing';
    recordingIndicator.classList.remove('active');
    
    clearInterval(recordingTimer);
    recordingTimerDisplay.textContent = '00:00';
    
    mediaRecorder.stop();
    
    // Reset button after processing
    setTimeout(() => {
        recordButton.className = 'record-button idle';
    }, 1000);
}

function startHandRecording() {
    console.log(getSelectedDevice())
    console.log(userPerMissionId)

    if (getSelectedDevice() != userPerMissionId){
        showError("Please select the correct device to start recording.")
        // alert('Please select the correct device to start recording.');
        return;
    }

    // Stop any currently playing audio
    stopCurrentAudio();

    isRecordingHand = true;
    recordHandButton.className = 'record-hand-button recording';
    recordingIndicator.classList.add('active');
    recordingStartTime = Date.now();

    // Start recording timer
    recordingTimer = setInterval(updateRecordingTimer, 100);

    // mediaRecorder.start();
}
function stopHandRecording() {
    // if (!isRecordingHand) return;

    isRecordingHand = false;
    recordHandButton.className = 'record-hand-button idle';
    recordingIndicator.classList.remove('active');

    clearInterval(recordingTimer);
    recordingTimerDisplay.textContent = '00:00';

    // mediaRecorder.stop();

    // Reset button after processing
    setTimeout(() => {
        recordHandButton.className = 'record-hand-button idle';
    }, 1000);
}

// Update recording timer
function updateRecordingTimer() {
    if (!recordingStartTime) return;
    
    const elapsed = Math.floor((Date.now() - recordingStartTime) / 1000);
    const minutes = Math.floor(elapsed / 60).toString().padStart(2, '0');
    const seconds = (elapsed % 60).toString().padStart(2, '0');
    recordingTimerDisplay.textContent = `${minutes}:${seconds}`;
}

// Send text message
function sendMessage() {
    const text = messageInput.value.trim();
    if (!text) return;
    
    // Stop any currently playing audio
    stopCurrentAudio();
    
    addMessage(text, true);
    messageInput.value = '';
    
    // Simulate bot response
    setTimeout(() => {
        const responses = [
            "I received your message. How can I assist you further?",
            "That's interesting! Tell me more about what you need.",
            "I'm processing your request. Please wait a moment...",
            "Thanks for your input! Here's what I found:",
            "I understand. Let me help you with that."
        ];
        const randomResponse = responses[Math.floor(Math.random() * responses.length)];
        addMessage(randomResponse);
    }, 1000);
}

// Event listeners
document.addEventListener('DOMContentLoaded', () => {
    // Close sidebar when overlay is clicked (mobile)
    sidebarOverlay.addEventListener('click', () => {
        if (window.innerWidth <= 768) {
            sidebar.classList.remove('active');
            sidebarOverlay.classList.remove('active');
            document.body.style.overflow = '';
        }
    });

    // Close sidebar when clicking outside on mobile
    document.addEventListener('click', (e) => {
        if (window.innerWidth <= 768) {
            const isClickInsideSidebar = sidebar.contains(e.target);
            const isToggleButton = e.target.closest('.toggle-btn');
            const isSidebarActive = sidebar.classList.contains('active');
            
            if (!isClickInsideSidebar && !isToggleButton && isSidebarActive) {
                sidebar.classList.remove('active');
                sidebarOverlay.classList.remove('active');
                document.body.style.overflow = '';
            }
        }
    });

    // Handle escape key to close sidebar on mobile
    document.addEventListener('keydown', (e) => {
        if (e.key === 'Escape' && window.innerWidth <= 768 && sidebar.classList.contains('active')) {
            sidebar.classList.remove('active');
            sidebarOverlay.classList.remove('active');
            document.body.style.overflow = '';
        }
    });

    recordHandButton.addEventListener('click', () => {
        if (isRecordingHand) {
            stopHandRecording();
        } else {
            startHandRecording();
        }
    });

    recordButton.addEventListener('click', () => {
        if (isRecording) {
            stopRecording();
        } else {
            startRecording();
        }
    });

    sendButton.addEventListener('click', sendMessage);

    messageInput.addEventListener('keypress', (e) => {
        if (e.key === 'Enter') {
            sendMessage();
        }
    });

    // Navigation click handlers
    document.querySelectorAll('.nav-link').forEach(link => {
        link.addEventListener('click', function(e) {
            e.preventDefault();
            document.querySelectorAll('.nav-link').forEach(l => l.classList.remove('active'));
            this.classList.add('active');
            console.log('Navigated to:', this.textContent.trim());
            
            // Close sidebar on mobile after navigation
            if (window.innerWidth <= 768) {
                sidebar.classList.remove('active');
                sidebarOverlay.classList.remove('active');
                document.body.style.overflow = '';
            }
        });
    });

    // Mobile responsiveness
    function handleResize() {
        if (window.innerWidth <= 768) {
            // Mobile mode
            sidebar.classList.remove('collapsed');
            sidebar.classList.remove('active'); // Ensure sidebar is hidden initially on mobile
            mainContent.classList.add('expanded');
            sidebarOverlay.classList.remove('active');
            document.body.style.overflow = '';
        } else {
            // Desktop mode
            sidebar.classList.remove('active');
            sidebarOverlay.classList.remove('active');
            document.body.style.overflow = '';
            
            if (!sidebarCollapsed) {
                sidebar.classList.remove('collapsed');
                mainContent.classList.remove('expanded');
            } else {
                sidebar.classList.add('collapsed');
                mainContent.classList.add('expanded');
            }
        }
    }

    window.addEventListener('resize', handleResize);

    // Initialize on page load
    handleResize();
    initializeAudio();

    // Prevent body scroll when sidebar is open on mobile
    function preventBodyScroll(e) {
        if (sidebar.classList.contains('active') && window.innerWidth <= 768) {
            if (!sidebar.contains(e.target)) {
                e.preventDefault();
            }
        }
    }

    document.addEventListener('touchmove', preventBodyScroll, { passive: false });

    // WebSocket connection
    let wsProtocol = window.location.protocol === "https:" ? "wss:" : "ws:";
    let wsHost = window.location.host;  // fdf7484d6f96.ngrok-free.app

    const ws = new WebSocket(`${wsProtocol}//${wsHost}/ws`);
    ws.onmessage = function(event) {
        const data = JSON.parse(event.data);
        if (data.type === "bot") {
            addMessage(data.content, false); // false = not user
        }
    
        if (data.type === "bot_audio") {
            // Base64 decode to binary
            const binaryString = atob(data.content);
            const len = binaryString.length;
            const bytes = new Uint8Array(len);
            for (let i = 0; i < len; i++) {
                bytes[i] = binaryString.charCodeAt(i);
            }
            const audioBlob = new Blob([bytes.buffer], { type: "audio/wav" });

            // Auto-play
            const audioUrl = URL.createObjectURL(audioBlob);
            const autoPlay = new Audio(audioUrl);
            autoPlay.play();

            // Add message with replay controls
            addMessage("", false, true, audioBlob);
        }

    };
});

// Cleanup when page unloads
window.addEventListener('beforeunload', () => {
    // Stop any playing audio and cleanup resources
    if (currentAudio) {
        currentAudio.pause();
        currentAudio = null;
    }
    
    audioInstances.forEach(audio => {
        audio.pause();
        audio.src = '';
    });
    audioInstances.clear();
    
    // Cleanup recording
    if (mediaRecorder && mediaRecorder.state !== 'inactive') {
        mediaRecorder.stop();
    }
    
    // Reset body overflow
    document.body.style.overflow = '';
});

class NotificationSystem {
    constructor() {
        this.container = document.getElementById('notificationContainer');
    }

    show(message, duration = 3000) {
        // Create notification element
        const notification = document.createElement('div');
        notification.className = 'notification';
        
        notification.innerHTML = `
            <div class="notification-icon">⚠️</div>
            <div>${message}</div>
            <div class="notification-close">&times;</div>
        `;

        // Add to container
        this.container.appendChild(notification);

        // Trigger animation
        setTimeout(() => {
            notification.classList.add('show');
        }, 100);

        // Auto-hide after duration
        const timeout = setTimeout(() => {
            this.hide(notification);
        }, duration);

        // Close button handler
        const closeButton = notification.querySelector('.notification-close');
        closeButton.addEventListener('click', () => {
            clearTimeout(timeout);
            this.hide(notification);
        });
    }

    hide(notification) {
        notification.classList.remove('show');
        setTimeout(() => {
            if (notification.parentNode) {
                notification.parentNode.removeChild(notification);
            }
        }, 300);
    }
}

// Initialize notification system
const notificationSystem = new NotificationSystem();

// Usage example in your error handling
async function handleAudioRecording(audioBlob) {
    try {
        addMessage('', true, true, audioBlob);
        const arrayBuffer = await audioBlob.arrayBuffer();
        const response = await fetch("/upload-audio", {
            method: "POST",
            headers: {
                "Content-Type": "audio/wav"
            },
            body: arrayBuffer
        });

        const result = await response.json();
        console.log("Server response:", result);

        addMessage(result.info, true);

    } catch (err) {
        // Use notification system instead of adding to chat
        notificationSystem.show("There was an error uploading your audio. Please try again.");
        console.error("Upload failed:", err);
    }
}

// Example usage elsewhere
function showError(message) {
    notificationSystem.show(message, 5000);
}


function getSelectedDevice() {
    const selectElement = document.getElementById('deviceSelect');
    return selectElement.value;
}

function createAudioPlayer(audioBlob, messageId) {
    const url = URL.createObjectURL(audioBlob);
    return `
        <div class="audio-message">
            <audio id="${messageId}" controls>
                <source src="${url}" type="audio/wav">
                Your browser does not support audio.
            </audio>
        </div>
    `;
}