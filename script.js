document.addEventListener('DOMContentLoaded', () => {
    // ==================== NAVIGATION & CORE UI ====================
    const navToggle = document.getElementById('nav-toggle');
    const navLinks = document.querySelector('.nav-links');
    const navLinksItems = document.querySelectorAll('.nav-link');
    const navbar = document.getElementById('navbar');
    const sections = document.querySelectorAll('section');
    
    // Mobile navigation toggle
    if (navToggle && navLinks) {
        navToggle.addEventListener('click', () => {
            navToggle.classList.toggle('active');
            navLinks.classList.toggle('active');
        });

        navLinksItems.forEach(item => {
            item.addEventListener('click', () => {
                navToggle.classList.remove('active');
                navLinks.classList.remove('active');
            });
        });
    }

    // Scroll handlers (sticky navbar & active links)
    window.addEventListener('scroll', () => {
        if (window.scrollY > 50) {
            navbar.classList.add('scrolled');
        } else {
            navbar.classList.remove('scrolled');
        }

        let currentSectionId = '';
        sections.forEach(section => {
            const sectionTop = section.offsetTop - 120;
            const sectionHeight = section.clientHeight;
            if (window.scrollY >= sectionTop && window.scrollY < sectionTop + sectionHeight) {
                currentSectionId = section.getAttribute('id');
            }
        });

        navLinksItems.forEach(link => {
            link.classList.remove('active');
            if (link.getAttribute('href') === `#${currentSectionId}`) {
                link.classList.add('active');
            }
        });
    });

    // Project Cards Accordion Toggle
    const projectCards = document.querySelectorAll('.project-card');
    projectCards.forEach(card => {
        const toggleBtn = card.querySelector('.project-toggle-btn');
        if (toggleBtn) {
            toggleBtn.addEventListener('click', () => {
                const isActive = card.classList.contains('active');
                card.classList.toggle('active');
                toggleBtn.textContent = isActive ? '展开详情' : '收起详情';
            });
        }
    });

    // Contact Form Real Submission using FormSubmit AJAX API
    const contactForm = document.getElementById('contact-form');
    const formMessage = document.getElementById('form-message');
    if (contactForm && formMessage) {
        contactForm.addEventListener('submit', (e) => {
            e.preventDefault();
            const name = document.getElementById('name').value;
            const email = document.getElementById('email').value;
            const message = document.getElementById('message').value;
            
            formMessage.style.display = 'block';
            formMessage.className = 'form-message';
            formMessage.style.backgroundColor = 'rgba(59, 130, 246, 0.15)';
            formMessage.style.color = '#3b82f6';
            formMessage.style.borderColor = 'rgba(59, 130, 246, 0.25)';
            formMessage.textContent = '正在发送您的消息，请稍候...';

            fetch('https://formsubmit.co/ajax/1357570890@qq.com', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                    'Accept': 'application/json'
                },
                body: JSON.stringify({
                    Name: name,
                    Email: email,
                    Message: message,
                    _subject: `来自个人简历主页的留言 - ${name}`
                })
            })
            .then(response => response.json())
            .then(data => {
                if (data.success === "true" || data.success === true) {
                    formMessage.className = 'form-message success';
                    formMessage.style.backgroundColor = '';
                    formMessage.style.color = '';
                    formMessage.style.borderColor = '';
                    formMessage.textContent = `感谢您的留言，${name}！您的消息已成功投递，我将尽快通过 ${email} 与您取得联系！`;
                    contactForm.reset();
                } else {
                    throw new Error('发送失败');
                }
            })
            .catch(error => {
                formMessage.className = 'form-message';
                formMessage.style.backgroundColor = 'rgba(239, 68, 68, 0.15)';
                formMessage.style.color = '#ef4444';
                formMessage.style.borderColor = 'rgba(239, 68, 68, 0.25)';
                formMessage.textContent = '抱歉，邮件发送失败，网络连接异常。请直接通过本页邮箱或电话与我取得联系。';
            });
        });
    }

    // Animating skill bars on load
    const skillBars = document.querySelectorAll('.skill-bar-fill');
    const animateSkills = () => {
        skillBars.forEach(bar => {
            const rect = bar.getBoundingClientRect();
            const viewHeight = Math.max(document.documentElement.clientHeight, window.innerHeight);
            if (!(rect.bottom < 0 || rect.top - viewHeight >= 0)) {
                bar.style.transform = 'scaleX(1)';
                bar.style.transition = 'transform 1.5s cubic-bezier(0.1, 1, 0.1, 1)';
            }
        });
    };
    window.addEventListener('scroll', animateSkills);
    animateSkills();

    // ==================== LAB TAB SWITCHING ====================
    const tabButtons = document.querySelectorAll('.lab-tab');
    const panels = document.querySelectorAll('.lab-panel');
    
    tabButtons.forEach(btn => {
        btn.addEventListener('click', () => {
            const targetTab = btn.getAttribute('data-tab');
            
            tabButtons.forEach(b => b.classList.remove('active'));
            panels.forEach(p => p.classList.remove('active'));
            
            btn.classList.add('active');
            document.getElementById(targetTab).classList.add('active');
            
            // Trigger redraw or initialization for the active panel
            if (targetTab === 'rrt-tab') initRRT();
            if (targetTab === 'pid-tab') initPID();
            if (targetTab === 'lidar-tab') initLIDAR();
            if (targetTab === 'csi-tab') initCSI();
        });
    });

    // ==================== RRT* PATH PLANNER ALGORITHM SIMULATION ====================
    let rrtCanvas, rrtCtx;
    let rrtNodes = [];
    let rrtObstacles = [];
    let rrtStart = { x: 50, y: 175 };
    let rrtGoal = { x: 500, y: 175 };
    let rrtGoalRadius = 15;
    let rrtStepSize = 20;
    let rrtSearchRadius = 35; // RRT* Rewire radius
    let rrtRunning = false;
    let rrtAnimationId = null;
    let isDrawingObstacle = false;
    let lastMousePos = null;

    function initRRT() {
        rrtCanvas = document.getElementById('rrt-canvas');
        if (!rrtCanvas) return;
        rrtCtx = rrtCanvas.getContext('2d');
        
        // Setup initial default obstacles
        rrtObstacles = [
            { x: 180, y: 30, w: 40, h: 180 },
            { x: 300, y: 140, w: 45, h: 180 },
            { x: 420, y: 50, w: 35, h: 160 }
        ];
        
        rrtNodes = [{ x: rrtStart.x, y: rrtStart.y, parent: null, cost: 0 }];
        rrtRunning = false;
        if (rrtAnimationId) cancelAnimationFrame(rrtAnimationId);
        
        drawRRT();
        
        // Mouse drawing for custom obstacles
        rrtCanvas.addEventListener('mousedown', startDrawingObstacle);
        rrtCanvas.addEventListener('mousemove', drawObstacleHandler);
        window.addEventListener('mouseup', stopDrawingObstacle);
    }

    function startDrawingObstacle(e) {
        const rect = rrtCanvas.getBoundingClientRect();
        const x = (e.clientX - rect.left) * (rrtCanvas.width / rect.width);
        const y = (e.clientY - rect.top) * (rrtCanvas.height / rect.height);
        
        // Check if clicked too close to start or goal
        if (dist(x, y, rrtStart.x, rrtStart.y) < 25 || dist(x, y, rrtGoal.x, rrtGoal.y) < 25) return;
        
        isDrawingObstacle = true;
        lastMousePos = { x, y };
        rrtObstacles.push({ x: x, y: y, w: 5, h: 5 });
    }

    function drawObstacleHandler(e) {
        if (!isDrawingObstacle) return;
        const rect = rrtCanvas.getBoundingClientRect();
        const x = (e.clientX - rect.left) * (rrtCanvas.width / rect.width);
        const y = (e.clientY - rect.top) * (rrtCanvas.height / rect.height);
        
        const currentOb = rrtObstacles[rrtObstacles.length - 1];
        
        // Draw rectangle from starting click to current pos
        currentOb.w = x - currentOb.x;
        currentOb.h = y - currentOb.y;
        
        drawRRT();
    }

    function stopDrawingObstacle() {
        if (isDrawingObstacle) {
            isDrawingObstacle = false;
            // Normalize obstacles with negative width/height
            const ob = rrtObstacles[rrtObstacles.length - 1];
            if (ob.w < 0) { ob.x += ob.w; ob.w = Math.abs(ob.w); }
            if (ob.h < 0) { ob.y += ob.h; ob.h = Math.abs(ob.h); }
            
            // Remove tiny accidental obstacles
            if (ob.w < 5 && ob.h < 5) {
                rrtObstacles.pop();
            }
            drawRRT();
        }
    }

    // Helper functions
    function dist(x1, y1, x2, y2) {
        return Math.sqrt((x1 - x2)**2 + (y1 - y2)**2);
    }

    function checkCollision(x1, y1, x2, y2) {
        // Line collision with AABB boxes
        for (let ob of rrtObstacles) {
            if (lineRectIntersect(x1, y1, x2, y2, ob.x, ob.y, ob.x + ob.w, ob.y + ob.h)) {
                return true;
            }
        }
        return false;
    }

    function lineRectIntersect(x1, y1, x2, y2, rx1, ry1, rx2, ry2) {
        // Liang-Barsky line clipping algorithm helper
        let t0 = 0, t1 = 1;
        let dx = x2 - x1;
        let dy = y2 - y1;
        
        let p = [-dx, dx, -dy, dy];
        let q = [x1 - rx1, rx2 - x1, y1 - ry1, ry2 - y1];
        
        for (let i = 0; i < 4; i++) {
            if (p[i] === 0) {
                if (q[i] < 0) return false;
            } else {
                let t = q[i] / p[i];
                if (p[i] < 0) {
                    if (t > t1) return false;
                    if (t > t0) t0 = t;
                } else {
                    if (t < t0) return false;
                    if (t < t1) t1 = t;
                }
            }
        }
        return t0 <= t1;
    }

    function stepRRT() {
        if (!rrtRunning) return;
        
        // 1. Sample random configuration
        let qRand = { x: Math.random() * rrtCanvas.width, y: Math.random() * rrtCanvas.height };
        // 5% bias towards goal to speed up demo
        if (Math.random() < 0.08) qRand = { x: rrtGoal.x, y: rrtGoal.y };
        
        // 2. Find nearest node
        let minD = Infinity;
        let qNear = null;
        for (let node of rrtNodes) {
            let d = dist(node.x, node.y, qRand.x, qRand.y);
            if (d < minD) {
                minD = d;
                qNear = node;
            }
        }
        
        // 3. Step towards qRand
        let angle = Math.atan2(qRand.y - qNear.y, qRand.x - qNear.x);
        let qNew = {
            x: qNear.x + Math.cos(angle) * rrtStepSize,
            y: qNear.y + Math.sin(angle) * rrtStepSize,
            parent: qNear,
            cost: qNear.cost + rrtStepSize
        };
        
        // Bound checks
        if (qNew.x < 0 || qNew.x > rrtCanvas.width || qNew.y < 0 || qNew.y > rrtCanvas.height) return;
        
        // 4. Collision check
        if (!checkCollision(qNear.x, qNear.y, qNew.x, qNew.y)) {
            
            // RRT* Optimization: Rewire tree in nearby radius
            let nearNodes = [];
            for (let node of rrtNodes) {
                if (dist(node.x, node.y, qNew.x, qNew.y) < rrtSearchRadius) {
                    nearNodes.push(node);
                }
            }
            
            // Find parent with minimum cost
            let qMin = qNear;
            let minCost = qNew.cost;
            for (let node of nearNodes) {
                let costWithNode = node.cost + dist(node.x, node.y, qNew.x, qNew.y);
                if (costWithNode < minCost && !checkCollision(node.x, node.y, qNew.x, qNew.y)) {
                    qMin = node;
                    minCost = costWithNode;
                }
            }
            
            qNew.parent = qMin;
            qNew.cost = minCost;
            rrtNodes.push(qNew);
            
            // Rewire other nodes to see if going through qNew is cheaper
            for (let node of nearNodes) {
                let costThroughNew = qNew.cost + dist(node.x, node.y, qNew.x, qNew.y);
                if (costThroughNew < node.cost && !checkCollision(qNew.x, qNew.y, node.x, node.y)) {
                    node.parent = qNew;
                    node.cost = costThroughNew;
                }
            }
            
            // Check if goal reached
            let distToGoal = dist(qNew.x, qNew.y, rrtGoal.x, rrtGoal.y);
            if (distToGoal < rrtGoalRadius) {
                document.getElementById('rrt-status').textContent = '规划成功 (已收敛)';
                document.getElementById('rrt-status').className = 'status-value status-success';
                rrtRunning = false;
            }
        }
    }

    function drawRRT() {
        rrtCtx.clearRect(0, 0, rrtCanvas.width, rrtCanvas.height);
        
        // Grid background
        rrtCtx.strokeStyle = 'rgba(255, 255, 255, 0.02)';
        rrtCtx.lineWidth = 1;
        for (let i = 0; i < rrtCanvas.width; i += 20) {
            rrtCtx.beginPath(); rrtCtx.moveTo(i, 0); rrtCtx.lineTo(i, rrtCanvas.height); rrtCtx.stroke();
        }
        for (let j = 0; j < rrtCanvas.height; j += 20) {
            rrtCtx.beginPath(); rrtCtx.moveTo(0, j); rrtCtx.lineTo(rrtCanvas.width, j); rrtCtx.stroke();
        }
        
        // Draw Obstacles
        rrtCtx.fillStyle = 'rgba(100, 116, 139, 0.6)';
        rrtCtx.strokeStyle = 'rgba(148, 163, 184, 0.8)';
        rrtCtx.lineWidth = 2;
        for (let ob of rrtObstacles) {
            rrtCtx.fillRect(ob.x, ob.y, ob.w, ob.h);
            rrtCtx.strokeRect(ob.x, ob.y, ob.w, ob.h);
        }
        
        // Draw Tree Nodes and Lines
        rrtCtx.strokeStyle = 'rgba(6, 182, 212, 0.35)';
        rrtCtx.lineWidth = 1;
        for (let node of rrtNodes) {
            if (node.parent) {
                rrtCtx.beginPath();
                rrtCtx.moveTo(node.x, node.y);
                rrtCtx.lineTo(node.parent.x, node.parent.y);
                rrtCtx.stroke();
            }
        }
        
        // Check if goal has a connection in tree
        let bestGoalNode = null;
        let minGoalCost = Infinity;
        for (let node of rrtNodes) {
            if (dist(node.x, node.y, rrtGoal.x, rrtGoal.y) < rrtGoalRadius) {
                if (node.cost < minGoalCost) {
                    minGoalCost = node.cost;
                    bestGoalNode = node;
                }
            }
        }
        
        // Draw path if found
        if (bestGoalNode) {
            rrtCtx.strokeStyle = '#f59e0b';
            rrtCtx.lineWidth = 3.5;
            rrtCtx.shadowColor = '#f59e0b';
            rrtCtx.shadowBlur = 8;
            rrtCtx.beginPath();
            rrtCtx.moveTo(rrtGoal.x, rrtGoal.y);
            let curr = bestGoalNode;
            while (curr) {
                rrtCtx.lineTo(curr.x, curr.y);
                curr = curr.parent;
            }
            rrtCtx.stroke();
            rrtCtx.shadowBlur = 0; // Reset
        }
        
        // Draw Start Node
        rrtCtx.fillStyle = '#10b981';
        rrtCtx.beginPath(); rrtCtx.arc(rrtStart.x, rrtStart.y, 7, 0, Math.PI * 2); rrtCtx.fill();
        rrtCtx.strokeStyle = '#ffffff'; rrtCtx.lineWidth = 2; rrtCtx.stroke();
        
        // Draw Goal Node
        rrtCtx.fillStyle = '#ef4444';
        rrtCtx.beginPath(); rrtCtx.arc(rrtGoal.x, rrtGoal.y, rrtGoalRadius, 0, Math.PI * 2); rrtCtx.fill();
        rrtCtx.strokeStyle = '#ffffff'; rrtCtx.lineWidth = 2; rrtCtx.stroke();
    }

    function rrtLoop() {
        if (!rrtRunning) return;
        // Do multiple steps per frame to make it faster
        for (let i = 0; i < 4; i++) {
            stepRRT();
        }
        drawRRT();
        rrtAnimationId = requestAnimationFrame(rrtLoop);
    }

    // Bind event listeners for RRT
    const rrtStartBtn = document.getElementById('rrt-start');
    if (rrtStartBtn) {
        rrtStartBtn.addEventListener('click', () => {
            if (rrtRunning) {
                rrtRunning = false;
                rrtStartBtn.textContent = '运行规划';
                document.getElementById('rrt-status').textContent = '已暂停';
                document.getElementById('rrt-status').className = 'status-value status-ready';
            } else {
                rrtRunning = true;
                rrtStartBtn.textContent = '暂停规划';
                document.getElementById('rrt-status').textContent = '探索中...';
                document.getElementById('rrt-status').className = 'status-value status-running';
                rrtLoop();
            }
        });
    }

    const rrtClearBtn = document.getElementById('rrt-clear-ob');
    if (rrtClearBtn) {
        rrtClearBtn.addEventListener('click', () => {
            rrtObstacles = [];
            initRRT();
        });
    }

    const rrtResetBtn = document.getElementById('rrt-reset');
    if (rrtResetBtn) {
        rrtResetBtn.addEventListener('click', () => {
            initRRT();
            rrtStartBtn.textContent = '运行规划';
            document.getElementById('rrt-status').textContent = '待就绪';
            document.getElementById('rrt-status').className = 'status-value status-ready';
        });
    }


    // ==================== PID CONTROLLER SYSTEMS SIMULATION ====================
    let pidCanvas, pidCtx;
    let pidTargetY = 150;      // Desired position
    let pidDroneY = 280;       // Current position
    let pidVelocity = 0;
    let pidErrLast = 0;
    let pidErrIntegral = 0;
    let pidTimeStep = 0.1;
    let pidHistory = [];       // For real-time graphing
    let pidAnimationId = null;

    // Controllers sliders elements
    let kpSlider, kiSlider, kdSlider;
    let kpDisplay, kiDisplay, kdDisplay;

    function initPID() {
        pidCanvas = document.getElementById('pid-canvas');
        if (!pidCanvas) return;
        pidCtx = pidCanvas.getContext('2d');
        
        kpSlider = document.getElementById('pid-kp');
        kiSlider = document.getElementById('pid-ki');
        kdSlider = document.getElementById('pid-kd');
        
        kpDisplay = document.getElementById('kp-val');
        kiDisplay = document.getElementById('ki-val');
        kdDisplay = document.getElementById('kd-val');
        
        pidDroneY = 280;
        pidVelocity = 0;
        pidErrLast = 0;
        pidErrIntegral = 0;
        pidHistory = Array(120).fill(280); // Empty graph history
        
        if (pidAnimationId) cancelAnimationFrame(pidAnimationId);
        
        // Listen to mouse clicks to set target height
        pidCanvas.addEventListener('mousedown', (e) => {
            const rect = pidCanvas.getBoundingClientRect();
            const y = (e.clientY - rect.top) * (pidCanvas.height / rect.height);
            // Cap to reasonable bounds
            if (y > 30 && y < 310) {
                pidTargetY = y;
            }
        });
        
        pidLoop();
    }

    function pidLoop() {
        // Read Slider values
        const kp = parseFloat(kpSlider.value);
        const ki = parseFloat(kiSlider.value);
        const kd = parseFloat(kdSlider.value);
        
        kpDisplay.textContent = kp.toFixed(2);
        kiDisplay.textContent = ki.toFixed(3);
        kdDisplay.textContent = kd.toFixed(2);
        
        // PID Formula calculation
        // Height direction is inverted in screen coordinates: y decreases as we go up.
        // Let's compute height from bottom:
        const currentHeight = pidCanvas.height - pidDroneY;
        const targetHeight = pidCanvas.height - pidTargetY;
        
        const error = targetHeight - currentHeight;
        
        // Proportional term
        const pTerm = kp * error;
        
        // Integral term (with windup clamp)
        pidErrIntegral += error * pidTimeStep;
        pidErrIntegral = Math.max(-50, Math.min(50, pidErrIntegral)); // Anti-windup
        const iTerm = ki * pidErrIntegral;
        
        // Derivative term
        const dTerm = kd * (error - pidErrLast) / pidTimeStep;
        pidErrLast = error;
        
        // Total control signal (thrust output force)
        const controlSignal = pTerm + iTerm + dTerm;
        
        // Physics integration
        const mass = 1.0;
        const gravity = 0.6; // downward force
        
        // Output acceleration: thrust - gravity
        // We cap thrust to reasonable bounds
        const thrust = Math.max(0, Math.min(2.5, controlSignal + gravity)); 
        const acceleration = thrust - gravity;
        
        pidVelocity += acceleration;
        pidDroneY -= pidVelocity; // Subtract because screen y is inverted
        
        // Floor and ceiling collisions
        if (pidDroneY > 320) {
            pidDroneY = 320;
            pidVelocity = 0;
        }
        if (pidDroneY < 20) {
            pidDroneY = 20;
            pidVelocity = 0;
        }
        
        // Record graph history
        pidHistory.push(pidDroneY);
        if (pidHistory.length > 150) pidHistory.shift();
        
        // Draw Frame
        drawPID(thrust);
        
        pidAnimationId = requestAnimationFrame(pidLoop);
    }

    function drawPID(thrust) {
        pidCtx.clearRect(0, 0, pidCanvas.width, pidCanvas.height);
        
        const graphLeft = 320;
        const graphWidth = 210;
        
        // Grid background for simulator space (Left pane)
        pidCtx.strokeStyle = 'rgba(255, 255, 255, 0.02)';
        pidCtx.lineWidth = 1;
        for (let i = 0; i < graphLeft; i += 20) {
            pidCtx.beginPath(); pidCtx.moveTo(i, 0); pidCtx.lineTo(i, pidCanvas.height); pidCtx.stroke();
        }
        for (let j = 0; j < pidCanvas.height; j += 20) {
            pidCtx.beginPath(); pidCtx.moveTo(0, j); pidCtx.lineTo(graphLeft, j); pidCtx.stroke();
        }
        
        // Separator between simulator and chart
        pidCtx.strokeStyle = 'rgba(255, 255, 255, 0.15)';
        pidCtx.lineWidth = 2;
        pidCtx.beginPath(); pidCtx.moveTo(graphLeft, 0); pidCtx.lineTo(graphLeft, pidCanvas.height); pidCtx.stroke();
        
        // --- 1. Draw Target Line (Red) ---
        pidCtx.strokeStyle = '#ef4444';
        pidCtx.lineWidth = 2;
        pidCtx.setLineDash([6, 4]);
        pidCtx.beginPath();
        pidCtx.moveTo(0, pidTargetY);
        pidCtx.lineTo(graphLeft, pidTargetY);
        pidCtx.stroke();
        pidCtx.setLineDash([]); // Reset
        
        pidCtx.fillStyle = '#ef4444';
        pidCtx.font = '10px monospace';
        pidCtx.fillText('SP (目标位置)', 10, pidTargetY - 5);
        
        // --- 2. Draw Drone ---
        const droneX = 160;
        const droneY = pidDroneY;
        
        // Thrust exhaust flame (Visualizing control output)
        if (thrust > 0.1) {
            const flameHeight = thrust * 18;
            const gradient = pidCtx.createLinearGradient(droneX, droneY + 5, droneX, droneY + 5 + flameHeight);
            gradient.addColorStop(0, 'rgba(251, 146, 60, 0.9)');
            gradient.addColorStop(0.5, 'rgba(249, 115, 22, 0.5)');
            gradient.addColorStop(1, 'rgba(239, 68, 68, 0)');
            pidCtx.fillStyle = gradient;
            pidCtx.beginPath();
            pidCtx.moveTo(droneX - 8, droneY + 5);
            pidCtx.lineTo(droneX + 8, droneY + 5);
            pidCtx.lineTo(droneX, droneY + 5 + flameHeight);
            pidCtx.closePath();
            pidCtx.fill();
        }
        
        // Drone body (cyan/silver)
        pidCtx.fillStyle = '#06b6d4';
        pidCtx.beginPath();
        pidCtx.arc(droneX, droneY, 10, 0, Math.PI * 2);
        pidCtx.fill();
        
        // Rotor arms
        pidCtx.strokeStyle = '#e2e8f0';
        pidCtx.lineWidth = 3;
        pidCtx.beginPath();
        pidCtx.moveTo(droneX - 25, droneY - 2);
        pidCtx.lineTo(droneX + 25, droneY - 2);
        pidCtx.stroke();
        
        // Rotors (horizontal lines representing spinning blades)
        pidCtx.strokeStyle = 'rgba(255, 255, 255, 0.4)';
        pidCtx.lineWidth = 1.5;
        // Left rotor
        pidCtx.beginPath();
        pidCtx.moveTo(droneX - 35, droneY - 8);
        pidCtx.lineTo(droneX - 15, droneY - 8);
        pidCtx.stroke();
        // Right rotor
        pidCtx.beginPath();
        pidCtx.moveTo(droneX + 15, droneY - 8);
        pidCtx.lineTo(droneX + 35, droneY - 8);
        pidCtx.stroke();
        
        // Vertical motor mounts
        pidCtx.strokeStyle = '#94a3b8';
        pidCtx.lineWidth = 2;
        pidCtx.beginPath();
        pidCtx.moveTo(droneX - 25, droneY - 2); pidCtx.lineTo(droneX - 25, droneY - 8);
        pidCtx.moveTo(droneX + 25, droneY - 2); pidCtx.lineTo(droneX + 25, droneY - 8);
        pidCtx.stroke();
        
        // --- 3. Draw Chart (Right pane) ---
        // Chart Background
        pidCtx.fillStyle = 'rgba(2, 6, 23, 0.4)';
        pidCtx.fillRect(graphLeft + 1, 0, graphWidth, pidCanvas.height);
        
        // Chart target reference (dashed red line)
        pidCtx.strokeStyle = 'rgba(239, 68, 68, 0.4)';
        pidCtx.lineWidth = 1.5;
        pidCtx.setLineDash([4, 4]);
        pidCtx.beginPath();
        pidCtx.moveTo(graphLeft, pidTargetY);
        pidCtx.lineTo(pidCanvas.width, pidTargetY);
        pidCtx.stroke();
        pidCtx.setLineDash([]);
        
        // Plot history values (Cyan line)
        pidCtx.strokeStyle = '#06b6d4';
        pidCtx.lineWidth = 2;
        pidCtx.beginPath();
        
        const step = graphWidth / 150;
        for (let i = 0; i < pidHistory.length; i++) {
            const lx = graphLeft + i * step;
            const ly = pidHistory[i];
            if (i === 0) {
                pidCtx.moveTo(lx, ly);
            } else {
                pidCtx.lineTo(lx, ly);
            }
        }
        pidCtx.stroke();
        
        // Real-time PV Tag
        pidCtx.fillStyle = '#06b6d4';
        pidCtx.font = '10px monospace';
        pidCtx.fillText('PV (实际位置)', graphLeft + 10, pidDroneY - 5);
        pidCtx.beginPath();
        pidCtx.arc(graphLeft + pidHistory.length * step, pidDroneY, 3, 0, Math.PI*2);
        pidCtx.fill();
    }

    // PID triggers
    const perturbBtn = document.getElementById('pid-perturb');
    if (perturbBtn) {
        perturbBtn.addEventListener('click', () => {
            // Apply instant upward/downward velocity kick
            pidVelocity += (Math.random() > 0.5 ? 4.5 : -4.5);
        });
    }

    const presetBtn = document.getElementById('pid-preset-optimal');
    if (presetBtn) {
        presetBtn.addEventListener('click', () => {
            kpSlider.value = 0.35;
            kiSlider.value = 0.005;
            kdSlider.value = 0.80;
            pidErrIntegral = 0;
        });
    }


    // ==================== 3D LIDAR MATRIX PROJECTION SIMULATION ====================
    let lidarCanvas, lidarCtx;
    let lidarPoints = [];
    let lidarAngleX = -0.3; // Tilt angle
    let lidarAngleY = 0.4;  // Rotation angle
    let isDraggingLidar = false;
    let previousMousePosition = { x: 0, y: 0 };
    let lidarAnimationId = null;

    function initLIDAR() {
        lidarCanvas = document.getElementById('lidar-canvas');
        if (!lidarCanvas) return;
        lidarCtx = lidarCanvas.getContext('2d');
        
        // Generate points representing a 3D robot dog frame skeleton
        generateLidarPoints();
        
        if (lidarAnimationId) cancelAnimationFrame(lidarAnimationId);
        
        // Drag listener to rotate point cloud
        lidarCanvas.addEventListener('mousedown', (e) => {
            isDraggingLidar = true;
            previousMousePosition = { x: e.clientX, y: e.clientY };
        });
        
        window.addEventListener('mousemove', (e) => {
            if (!isDraggingLidar) return;
            const deltaX = e.clientX - previousMousePosition.x;
            const deltaY = e.clientY - previousMousePosition.y;
            
            lidarAngleY += deltaX * 0.008; // Yaw
            lidarAngleX += deltaY * 0.008; // Pitch
            
            // Constrain pitch to avoid flipping upside down
            lidarAngleX = Math.max(-Math.PI / 2.5, Math.min(Math.PI / 2.5, lidarAngleX));
            
            previousMousePosition = { x: e.clientX, y: e.clientY };
            drawLidar();
        });
        
        window.addEventListener('mouseup', () => {
            isDraggingLidar = false;
        });
        
        lidarAutoRotateLoop();
    }

    function generateLidarPoints() {
        lidarPoints = [];
        
        // 1. Generate 3D Grid floor (LIDAR ground scan lines)
        for (let x = -80; x <= 80; x += 16) {
            for (let z = -80; z <= 80; z += 16) {
                lidarPoints.push({ x: x, y: 60, z: z, type: 'ground' });
            }
        }
        
        // 2. Generate a 3D Robot Dog skeleton (Point Cloud)
        // Dog Torso Box: Center at (0, 0, 0), dimensions (w=60, h=24, d=20)
        for (let x = -30; x <= 30; x += 4) {
            for (let y = -12; y <= 12; y += 8) {
                lidarPoints.push({ x: x, y: y, z: -10, type: 'dog' });
                lidarPoints.push({ x: x, y: y, z: 10, type: 'dog' });
            }
            for (let z = -10; z <= 10; z += 5) {
                lidarPoints.push({ x: x, y: -12, z: z, type: 'dog' });
                lidarPoints.push({ x: x, y: 12, z: z, type: 'dog' });
            }
        }
        
        // 4 Legs: Front-Left (30, y, 10), Front-Right (30, y, -10), etc.
        const legX = [-25, 25];
        const legZ = [-10, 10];
        
        for (let lx of legX) {
            for (let lz of legZ) {
                // Leg lines from body joint (y=12) down to foot (y=55)
                for (let ly = 12; ly <= 55; ly += 3) {
                    // Add some noise to mimic raw LIDAR scans
                    let rx = lx + (Math.random() - 0.5) * 1.5;
                    let rz = lz + (Math.random() - 0.5) * 1.5;
                    lidarPoints.push({ x: rx, y: ly, z: rz, type: 'leg' });
                }
                // Foot pads (larger cluster)
                for (let d = 0; d < 8; d++) {
                    lidarPoints.push({ 
                        x: lx + (Math.random() - 0.5) * 4, 
                        y: 56 + (Math.random() - 0.5) * 2, 
                        z: lz + (Math.random() - 0.5) * 4, 
                        type: 'leg' 
                    });
                }
            }
        }
        
        // Head / Sensor unit: at (38, -18, 0)
        for (let r = 0; r < 35; r++) {
            let hx = 35 + (Math.random() - 0.5) * 12;
            let hy = -20 + (Math.random() - 0.5) * 10;
            let hz = 0 + (Math.random() - 0.5) * 8;
            lidarPoints.push({ x: hx, y: hy, z: hz, type: 'head' });
        }
    }

    function rotate3D(point, angleX, angleY) {
        // Yaw rotation (around Y axis)
        const cosY = Math.cos(angleY);
        const sinY = Math.sin(angleY);
        let x1 = point.x * cosY - point.z * sinY;
        let z1 = point.x * sinY + point.z * cosY;
        
        // Pitch rotation (around X axis)
        const cosX = Math.cos(angleX);
        const sinX = Math.sin(angleX);
        let y2 = point.y * cosX - z1 * sinX;
        let z2 = point.y * sinX + z1 * cosX;
        
        return { x: x1, y: y2, z: z2, type: point.type };
    }

    function drawLidar() {
        lidarCtx.clearRect(0, 0, lidarCanvas.width, lidarCanvas.height);
        
        const cx = lidarCanvas.width / 2;
        const cy = lidarCanvas.height / 2;
        const fov = 350; // Camera perspective distance
        
        // Sort points by z-depth (Back-to-Front painter's algorithm for visual correctness)
        const rotatedPoints = lidarPoints.map(p => rotate3D(p, lidarAngleX, lidarAngleY));
        rotatedPoints.sort((a, b) => b.z - a.z);
        
        // Render projected points
        for (let p of rotatedPoints) {
            // Perspective projection formula
            // Camera distance: z + 200
            const zDistance = p.z + 200;
            if (zDistance <= 10) continue; // Clip behind camera
            
            const screenX = cx + (p.x * fov) / zDistance;
            const screenY = cy + (p.y * fov) / zDistance;
            
            // Render depth representation using point sizing and opacity
            const size = Math.max(1, (600 / zDistance));
            const opacity = Math.max(0.08, Math.min(1, 1 - (p.z + 80) / 200));
            
            if (p.type === 'ground') {
                lidarCtx.fillStyle = `rgba(30, 41, 59, ${opacity * 0.5})`;
                lidarCtx.beginPath();
                lidarCtx.arc(screenX, screenY, size * 0.7, 0, Math.PI * 2);
                lidarCtx.fill();
            } else {
                // Neon cyan color mapping for robot dog points
                let color = '6, 182, 212'; // Cyan
                if (p.type === 'head') color = '236, 72, 153'; // Pink head scanner
                if (p.type === 'leg') color = '59, 130, 246'; // Blue legs
                
                lidarCtx.fillStyle = `rgba(${color}, ${opacity})`;
                lidarCtx.beginPath();
                lidarCtx.arc(screenX, screenY, size, 0, Math.PI * 2);
                lidarCtx.fill();
                
                // Add tiny lidar beam halo for visual glow
                if (size > 2.5 && Math.random() < 0.25) {
                    lidarCtx.fillStyle = `rgba(${color}, ${opacity * 0.25})`;
                    lidarCtx.beginPath();
                    lidarCtx.arc(screenX, screenY, size * 2.2, 0, Math.PI * 2);
                    lidarCtx.fill();
                }
            }
        }
        
        // Draw HUD overlay in corner
        lidarCtx.fillStyle = 'rgba(6, 182, 212, 0.7)';
        lidarCtx.font = '10px monospace';
        lidarCtx.fillText(`[RADAR SCAN] Yaw: ${(lidarAngleY * 180 / Math.PI % 360).toFixed(0)}°  Pitch: ${(lidarAngleX * 180 / Math.PI).toFixed(0)}°`, 15, 25);
        lidarCtx.fillText(`[POINTS COUNT] Total: ${lidarPoints.length} scan points`, 15, 40);
        
        // Draw mouse rotation indicator
        lidarCtx.fillStyle = 'rgba(255, 255, 255, 0.25)';
        lidarCtx.fillText('按住鼠标拖拽可 3D 旋转视角', lidarCanvas.width - 150, lidarCanvas.height - 15);
    }

    function lidarAutoRotateLoop() {
        if (isDraggingLidar) return;
        
        // Drift rotate slowly
        lidarAngleY += 0.003;
        drawLidar();
        
        lidarAnimationId = requestAnimationFrame(lidarAutoRotateLoop);
    }

    // ==================== WI-FI CSI SIGNAL FILTERING SIMULATION ====================
    let csiCanvas, csiCtx;
    let csiRawData = [];
    let csiKalmanData = [];
    let csiMAData = [];
    let csiTime = 0;
    let csiAnimationId = null;
    
    // Sim parameters
    let csiQSlider, csiRSlider, csiWSlider;
    let csiQDisplay, csiRDisplay, csiWDisplay;
    let csiFilterMode = 'kalman'; // 'kalman' or 'ma'
    let csiPerturbTicks = 0;
    
    // Kalman variables
    let kX = 180; // Estimated state (amplitude)
    let kP = 1.0; // Estimate covariance

    function initCSI() {
        csiCanvas = document.getElementById('csi-canvas');
        if (!csiCanvas) return;
        csiCtx = csiCanvas.getContext('2d');
        
        csiQSlider = document.getElementById('csi-q');
        csiRSlider = document.getElementById('csi-r');
        csiWSlider = document.getElementById('csi-w');
        
        csiQDisplay = document.getElementById('csi-q-val');
        csiRDisplay = document.getElementById('csi-r-val');
        csiWDisplay = document.getElementById('csi-w-val');
        
        // Reset buffers
        csiRawData = Array(150).fill(180);
        csiKalmanData = Array(150).fill(180);
        csiMAData = Array(150).fill(180);
        csiTime = 0;
        kX = 180;
        kP = 1.0;
        csiPerturbTicks = 0;
        
        if (csiAnimationId) cancelAnimationFrame(csiAnimationId);
        
        csiLoop();
    }

    function csiLoop() {
        // Read parameters
        const q = parseFloat(csiQSlider.value);
        const r = parseFloat(csiRSlider.value);
        const w = parseInt(csiWSlider.value);
        
        csiQDisplay.textContent = q.toFixed(3);
        csiRDisplay.textContent = r.toFixed(2);
        csiWDisplay.textContent = w;
        
        // Generate raw noisy signal
        csiTime += 0.05;
        let baseVal = 180 + Math.sin(csiTime * 0.8) * 35 + Math.cos(csiTime * 0.3) * 15;
        
        // Simulating human fall spike: double peak anomaly
        if (csiPerturbTicks > 0) {
            const t = 40 - csiPerturbTicks;
            const spike = Math.sin(t * 0.2) * 80 + Math.sin(t * 0.5) * 30;
            baseVal += Math.abs(spike);
            csiPerturbTicks--;
        }
        
        const noise = (Math.random() - 0.5) * 30 + (Math.random() < 0.04 ? (Math.random() - 0.5) * 70 : 0); // Noise + outliers
        const measurement = baseVal + noise;
        
        csiRawData.push(measurement);
        if (csiRawData.length > 150) csiRawData.shift();
        
        // --- 1. 1D Kalman Filter Update ---
        const x_pred = kX;
        const p_pred = kP + q;
        const kGain = p_pred / (p_pred + r);
        kX = x_pred + kGain * (measurement - x_pred);
        kP = (1 - kGain) * p_pred;
        
        csiKalmanData.push(kX);
        if (csiKalmanData.length > 150) csiKalmanData.shift();
        
        // --- 2. Moving Average Filter Update ---
        let maSum = 0;
        const currentLength = csiRawData.length;
        const windowSize = Math.min(w, currentLength);
        for (let i = currentLength - windowSize; i < currentLength; i++) {
            maSum += csiRawData[i];
        }
        const maVal = maSum / windowSize;
        
        csiMAData.push(maVal);
        if (csiMAData.length > 150) csiMAData.shift();
        
        // Draw Frame
        drawCSI();
        
        csiAnimationId = requestAnimationFrame(csiLoop);
    }

    function drawCSI() {
        csiCtx.clearRect(0, 0, csiCanvas.width, csiCanvas.height);
        
        // Grid background
        csiCtx.strokeStyle = 'rgba(255, 255, 255, 0.02)';
        csiCtx.lineWidth = 1;
        for (let i = 0; i < csiCanvas.width; i += 20) {
            csiCtx.beginPath(); csiCtx.moveTo(i, 0); csiCtx.lineTo(i, csiCanvas.height); csiCtx.stroke();
        }
        for (let j = 0; j < csiCanvas.height; j += 20) {
            csiCtx.beginPath(); csiCtx.moveTo(0, j); csiCtx.lineTo(csiCanvas.width, j); csiCtx.stroke();
        }
        
        const step = csiCanvas.width / 150;
        
        // --- 1. Plot Raw Noisy Signal (Red) ---
        csiCtx.strokeStyle = 'rgba(239, 68, 68, 0.35)';
        csiCtx.lineWidth = 1.2;
        csiCtx.beginPath();
        for (let i = 0; i < csiRawData.length; i++) {
            const lx = i * step;
            const ly = csiRawData[i];
            if (i === 0) csiCtx.moveTo(lx, ly); else csiCtx.lineTo(lx, ly);
        }
        csiCtx.stroke();
        
        // --- 2. Plot Filtered Signal ---
        if (csiFilterMode === 'kalman') {
            csiCtx.strokeStyle = '#10b981';
            csiCtx.shadowColor = '#10b981';
            csiCtx.shadowBlur = 6;
            csiCtx.lineWidth = 2.5;
            csiCtx.beginPath();
            for (let i = 0; i < csiKalmanData.length; i++) {
                const lx = i * step;
                const ly = csiKalmanData[i];
                if (i === 0) csiCtx.moveTo(lx, ly); else csiCtx.lineTo(lx, ly);
            }
            csiCtx.stroke();
            csiCtx.shadowBlur = 0;
        } else {
            csiCtx.strokeStyle = '#f59e0b';
            csiCtx.shadowColor = '#f59e0b';
            csiCtx.shadowBlur = 6;
            csiCtx.lineWidth = 2.5;
            csiCtx.beginPath();
            for (let i = 0; i < csiMAData.length; i++) {
                const lx = i * step;
                const ly = csiMAData[i];
                if (i === 0) csiCtx.moveTo(lx, ly); else csiCtx.lineTo(lx, ly);
            }
            csiCtx.stroke();
            csiCtx.shadowBlur = 0;
        }
        
        // --- 3. Legend & HUD ---
        csiCtx.font = '10px monospace';
        csiCtx.fillStyle = 'rgba(239, 68, 68, 0.8)';
        csiCtx.fillRect(15, 20, 15, 8);
        csiCtx.fillText('原始采集 CSI 信号 (高频室内扰动与随机多径离群噪声)', 38, 27);
        
        if (csiFilterMode === 'kalman') {
            csiCtx.fillStyle = '#10b981';
            csiCtx.fillRect(15, 36, 15, 8);
            csiCtx.fillText('卡尔曼一维实时估计信号 (Kalman Filtered)', 38, 43);
        } else {
            csiCtx.fillStyle = '#f59e0b';
            csiCtx.fillRect(15, 36, 15, 8);
            csiCtx.fillText('窗口加权滑动平均信号 (Moving Average Filtered)', 38, 43);
        }
        
        csiCtx.fillStyle = 'rgba(255, 255, 255, 0.4)';
        csiCtx.fillText(`[信号频段] Wi-Fi 5.8GHz 信道子载波振幅幅值 (Amplitude)`, 15, csiCanvas.height - 15);
        
        if (csiPerturbTicks > 0) {
            csiCtx.fillStyle = '#ef4444';
            csiCtx.fillText(`[跌倒事件识别] 检测到高瞬态幅值陡变 (动态模拟跌倒中...)`, 15, csiCanvas.height - 30);
        }
    }

    const filterToggleBtn = document.getElementById('csi-filter-toggle');
    if (filterToggleBtn) {
        filterToggleBtn.addEventListener('click', () => {
            if (csiFilterMode === 'kalman') {
                csiFilterMode = 'ma';
                filterToggleBtn.textContent = '使用中: 滑动平均滤波';
            } else {
                csiFilterMode = 'kalman';
                filterToggleBtn.textContent = '使用中: 卡尔曼滤波';
            }
            drawCSI();
        });
    }

    const csiPerturbBtn = document.getElementById('csi-perturb');
    if (csiPerturbBtn) {
        csiPerturbBtn.addEventListener('click', () => {
            csiPerturbTicks = 40;
        });
    }

    const lidarResetBtn = document.getElementById('lidar-reset');
    if (lidarResetBtn) {
        lidarResetBtn.addEventListener('click', () => {
            lidarAngleX = -0.3;
            lidarAngleY = 0.4;
            drawLidar();
        });
    }

    // ==================== INITIALIZATION TRIGGER ====================
    // Run the active tab initially
    initRRT();

    // ==================== EASTER EGG (DEVELOPER CONSOLE) ====================
    console.log(
        "%c" + 
        "       __      _\n" +
        "     o'')}____//\n" +
        "      `_/      )\n" +
        "      (_(_/-(_/\n",
        "color: #06b6d4; font-weight: bold; font-family: monospace;"
    );
    console.log(
        "%c[张颢震 | 机器人与控制算法工程师 个人主页控制台]\n" +
        "%c>>> 检测到技术面面试官/HR访问。欢迎查看我手写的原生算法仿真！\n" +
        ">>> 本主页完全使用原生 JavaScript 构建，算法实验室核心功能：\n" +
        "    1. RRT* 路径搜索树（包含近邻 Rewire 路径渐近最优重构）\n" +
        "    2. 一维垂直高度 PID 稳定控制器控制环（包含积分抗饱和 anti-windup 限幅）\n" +
        "    3. 3D 旋转矩阵激光雷达点云变换投影（直接基于欧拉角旋转算子矩阵相乘）\n" +
        "\n" +
        ">>> 我的学术与技术标签：\n" +
        "    - 矩阵论考核成绩: 95分 | 随机过程: 89分 | 省级二等学业奖学金\n" +
        "    - 核心专业背景: C++ / Python / ROS / 3D LiDAR SLAM建图 / YOLO视觉检测\n" +
        "    - 联系方式: 电话/微信 183-9101-5680 | 邮箱 1357570890@qq.com\n" +
        "    - 期望城市: 西安 (寻求军工研究所/自动驾驶/机器人核心算法岗)\n" +
        "\n" +
        "如果您对我的底层数学或代码变现能力感兴趣，欢迎随时联系我！",
        "color: #3b82f6; font-size: 14px; font-weight: bold;",
        "color: #94a3b8; font-size: 12px;"
    );
});
