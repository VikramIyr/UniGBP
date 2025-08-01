// GBP Autonomy Stack Website JavaScript

document.addEventListener('DOMContentLoaded', function() {
    // Initialize all interactive features
    initNavigation();
    initScrollAnimations();
    initVideoPlaceholders();
    initParallaxEffect();
    initSmoothScrolling();
});

// Navigation functionality
function initNavigation() {
    const navbar = document.querySelector('.navbar');
    const navLinks = document.querySelectorAll('.nav-link');
    
    // Handle navbar background on scroll
    window.addEventListener('scroll', function() {
        if (window.scrollY > 50) {
            navbar.style.background = 'hsl(var(--background) / 0.98)';
            navbar.style.boxShadow = 'var(--shadow-sm)';
        } else {
            navbar.style.background = 'hsl(var(--background) / 0.95)';
            navbar.style.boxShadow = 'none';
        }
    });
    
    // Update active nav link based on scroll position
    window.addEventListener('scroll', function() {
        let current = '';
        const sections = document.querySelectorAll('section[id]');
        
        sections.forEach(section => {
            const sectionTop = section.offsetTop - 100;
            const sectionHeight = section.clientHeight;
            
            if (window.scrollY >= sectionTop && window.scrollY < sectionTop + sectionHeight) {
                current = section.getAttribute('id');
            }
        });
        
        navLinks.forEach(link => {
            link.classList.remove('active');
            if (link.getAttribute('href') === `#${current}`) {
                link.classList.add('active');
            }
        });
    });
}

// Scroll animations using Intersection Observer
function initScrollAnimations() {
    const observerOptions = {
        threshold: 0.1,
        rootMargin: '0px 0px -50px 0px'
    };
    
    const observer = new IntersectionObserver((entries) => {
        entries.forEach(entry => {
            if (entry.isIntersecting) {
                entry.target.style.opacity = '1';
                entry.target.style.transform = 'translateY(0)';
                
                // Special handling for staggered animations
                if (entry.target.classList.contains('feature-card') || 
                    entry.target.classList.contains('demo-card')) {
                    const delay = Array.from(entry.target.parentNode.children).indexOf(entry.target) * 100;
                    entry.target.style.transitionDelay = `${delay}ms`;
                }
            }
        });
    }, observerOptions);
    
    // Observe elements that should animate in
    const animatedElements = document.querySelectorAll(`
        .feature-card,
        .demo-card,
        .component-item,
        .method-item,
        .stat-item
    `);
    
    animatedElements.forEach(el => {
        el.style.opacity = '0';
        el.style.transform = 'translateY(30px)';
        el.style.transition = 'opacity 0.6s ease, transform 0.6s ease';
        observer.observe(el);
    });
}

// Video placeholder interactions
function initVideoPlaceholders() {
    const videoPlaceholders = document.querySelectorAll('.video-placeholder');
    
    videoPlaceholders.forEach(placeholder => {
        placeholder.addEventListener('click', function() {
            // Create ripple effect
            const ripple = document.createElement('div');
            ripple.style.position = 'absolute';
            ripple.style.borderRadius = '50%';
            ripple.style.background = 'rgba(255, 255, 255, 0.6)';
            ripple.style.transform = 'scale(0)';
            ripple.style.animation = 'ripple 0.6s linear';
            ripple.style.left = '50%';
            ripple.style.top = '50%';
            ripple.style.width = '20px';
            ripple.style.height = '20px';
            ripple.style.marginLeft = '-10px';
            ripple.style.marginTop = '-10px';
            
            placeholder.appendChild(ripple);
            
            // Remove ripple after animation
            setTimeout(() => {
                ripple.remove();
            }, 600);
            
            // Simulate video loading
            setTimeout(() => {
                showVideoModal(placeholder);
            }, 300);
        });
        
        // Add hover effect to play button
        const playButton = placeholder.querySelector('.play-button');
        placeholder.addEventListener('mouseenter', function() {
            playButton.style.transform = 'scale(1.1)';
        });
        
        placeholder.addEventListener('mouseleave', function() {
            playButton.style.transform = 'scale(1)';
        });
    });
}

// Show video modal (placeholder for actual video integration)
function showVideoModal(placeholder) {
    const modal = document.createElement('div');
    modal.style.position = 'fixed';
    modal.style.top = '0';
    modal.style.left = '0';
    modal.style.width = '100%';
    modal.style.height = '100%';
    modal.style.background = 'rgba(0, 0, 0, 0.9)';
    modal.style.display = 'flex';
    modal.style.alignItems = 'center';
    modal.style.justifyContent = 'center';
    modal.style.zIndex = '10000';
    modal.style.opacity = '0';
    modal.style.transition = 'opacity 0.3s ease';
    
    const videoContainer = document.createElement('div');
    videoContainer.style.width = '90%';
    videoContainer.style.maxWidth = '800px';
    videoContainer.style.aspectRatio = '16/9';
    videoContainer.style.background = 'var(--gradient-primary)';
    videoContainer.style.borderRadius = '12px';
    videoContainer.style.position = 'relative';
    videoContainer.style.display = 'flex';
    videoContainer.style.alignItems = 'center';
    videoContainer.style.justifyContent = 'center';
    videoContainer.style.color = 'white';
    videoContainer.style.fontSize = '1.2rem';
    videoContainer.innerHTML = `
        <div style="text-align: center;">
            <div style="font-size: 3rem; margin-bottom: 1rem;">🎬</div>
            <div>Demo Video Placeholder</div>
            <div style="font-size: 0.9rem; opacity: 0.8; margin-top: 0.5rem;">
                Replace this with your actual video content
            </div>
        </div>
    `;
    
    const closeButton = document.createElement('button');
    closeButton.innerHTML = '×';
    closeButton.style.position = 'absolute';
    closeButton.style.top = '20px';
    closeButton.style.right = '20px';
    closeButton.style.background = 'rgba(255, 255, 255, 0.2)';
    closeButton.style.border = 'none';
    closeButton.style.color = 'white';
    closeButton.style.fontSize = '2rem';
    closeButton.style.width = '50px';
    closeButton.style.height = '50px';
    closeButton.style.borderRadius = '50%';
    closeButton.style.cursor = 'pointer';
    closeButton.style.backdropFilter = 'blur(10px)';
    closeButton.style.transition = 'var(--transition-smooth)';
    
    closeButton.addEventListener('click', () => {
        modal.style.opacity = '0';
        setTimeout(() => modal.remove(), 300);
    });
    
    closeButton.addEventListener('mouseenter', () => {
        closeButton.style.background = 'rgba(255, 255, 255, 0.3)';
        closeButton.style.transform = 'scale(1.1)';
    });
    
    closeButton.addEventListener('mouseleave', () => {
        closeButton.style.background = 'rgba(255, 255, 255, 0.2)';
        closeButton.style.transform = 'scale(1)';
    });
    
    modal.addEventListener('click', (e) => {
        if (e.target === modal) {
            modal.style.opacity = '0';
            setTimeout(() => modal.remove(), 300);
        }
    });
    
    modal.appendChild(videoContainer);
    modal.appendChild(closeButton);
    document.body.appendChild(modal);
    
    // Fade in modal
    setTimeout(() => {
        modal.style.opacity = '1';
    }, 10);
}

// Parallax effect for hero background
function initParallaxEffect() {
    const heroBackground = document.querySelector('.hero-background');
    
    window.addEventListener('scroll', function() {
        const scrolled = window.pageYOffset;
        const parallax = scrolled * 0.5;
        
        if (heroBackground) {
            heroBackground.style.transform = `translate3d(0, ${parallax}px, 0)`;
        }
    });
}

// Smooth scrolling for navigation links
function initSmoothScrolling() {
    const navLinks = document.querySelectorAll('a[href^="#"]');
    
    navLinks.forEach(link => {
        link.addEventListener('click', function(e) {
            e.preventDefault();
            
            const targetId = this.getAttribute('href');
            const targetSection = document.querySelector(targetId);
            
            if (targetSection) {
                const offsetTop = targetSection.offsetTop - 70; // Account for navbar height
                
                window.scrollTo({
                    top: offsetTop,
                    behavior: 'smooth'
                });
            }
        });
    });
}

// Add CSS for ripple animation
const style = document.createElement('style');
style.textContent = `
    @keyframes ripple {
        to {
            transform: scale(4);
            opacity: 0;
        }
    }
    
    .nav-link.active {
        color: hsl(var(--accent)) !important;
    }
    
    .nav-link.active::after {
        width: 100% !important;
    }
`;
document.head.appendChild(style);

// Counter animation for stats
function animateCounters() {
    const counters = document.querySelectorAll('.stat-number');
    
    counters.forEach(counter => {
        const target = counter.textContent;
        const isNumber = /^\d+(\.\d+)?/.test(target);
        
        if (isNumber) {
            const finalNumber = parseFloat(target);
            let current = 0;
            const increment = finalNumber / 60; // 60 frames for 1 second at 60fps
            
            const updateCounter = () => {
                if (current < finalNumber) {
                    current += increment;
                    counter.textContent = Math.floor(current);
                    requestAnimationFrame(updateCounter);
                } else {
                    counter.textContent = target; // Ensure final value is exact
                }
            };
            
            updateCounter();
        }
    });
}

// Trigger counter animation when stats section is visible
const statsObserver = new IntersectionObserver((entries) => {
    entries.forEach(entry => {
        if (entry.isIntersecting) {
            animateCounters();
            statsObserver.unobserve(entry.target);
        }
    });
}, { threshold: 0.5 });

const statsSection = document.querySelector('.overview-stats');
if (statsSection) {
    statsObserver.observe(statsSection);
}

// Add loading states and micro-interactions
document.addEventListener('DOMContentLoaded', function() {
    // Add loading state to buttons
    const buttons = document.querySelectorAll('.btn');
    buttons.forEach(button => {
        button.addEventListener('click', function(e) {
            // Add subtle loading effect
            this.style.transform = 'scale(0.98)';
            setTimeout(() => {
                this.style.transform = '';
            }, 150);
        });
    });
    
    // Add hover effects to cards
    const cards = document.querySelectorAll('.feature-card, .demo-card, .component-item, .method-item');
    cards.forEach(card => {
        card.addEventListener('mouseenter', function() {
            this.style.transition = 'var(--transition-smooth)';
        });
    });
});

// Performance optimization: throttle scroll events
function throttle(func, limit) {
    let inThrottle;
    return function() {
        const args = arguments;
        const context = this;
        if (!inThrottle) {
            func.apply(context, args);
            inThrottle = true;
            setTimeout(() => inThrottle = false, limit);
        }
    }
}

// Apply throttling to scroll-heavy functions
window.addEventListener('scroll', throttle(function() {
    // Throttled scroll operations here
}, 16)); // ~60fps