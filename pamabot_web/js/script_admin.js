let localIp = location.hostname;

let moveIntervalId = null;


let data = {
    ros: null,
    rosbridge_address: `ws://${localIp}:9090`,
    connected: false,
    reverse: false,
    currentPosition: { x: 0, y: 0, z: 0, w: 1 }
};



function cancelNavigation() {
    if (moveIntervalId) {
        clearInterval(moveIntervalId);
        moveIntervalId = null;
        console.log(" Movimiento manual cancelado");
    }

    
    stop();

    
    const x = parseFloat(document.getElementById("pos_x").innerText);
    const y = parseFloat(document.getElementById("pos_y").innerText);
    const w = parseFloat(document.getElementById("pos_w").innerText);

    if (!isNaN(x) && !isNaN(y) && !isNaN(w)) {
        
        sendNavGoal(x, y, w);
        console.log(` Navegación cancelada reenviando objetivo a (${x.toFixed(2)}, ${y.toFixed(2)}, ${w.toFixed(2)})`);
    } else {
        console.warn(" Coordenadas actuales inválidas, no se puede cancelar la navegación.");
    }
}





let navGoalClient = null;

function sendNavGoal(x, y, w = 1.0) {
    if (!data.connected) {
        console.warn("No conectado");
        return;
    }

    const goalPublisher = new ROSLIB.Topic({
        ros: data.ros,
        name: '/goal_pose',
        messageType: 'geometry_msgs/msg/PoseStamped'
    });

    const goal = new ROSLIB.Message({
        header: {
            frame_id: "map"
        },
        pose: {
            position: {
                x: x,
                y: y,
                z: 0.0
            },
            orientation: {
                x: 0.0,
                y: 0.0,
                z: 0.0,
                w: w
            }
        }
    });

    console.log("🔵 Enviando objetivo a /goal_pose:", goal);
    goalPublisher.publish(goal);
}



function updateRosBridgeAddress() {
    const ipInput = document.getElementById("ipInput");
    if (ipInput) {
        data.rosbridge_address = ipInput.value || data.rosbridge_address;
    }
}


// Código de navegación
function navigateTo(targetX, targetY, targetOrientationW = 1.0) {
    if (!data.connected) {
        console.warn("No conectado");
        return;
    }

    const { x, y, z, w } = data.currentPosition;

    const cmdVelTopic = new ROSLIB.Topic({
        ros: data.ros,
        name: '/cmd_vel',
        messageType: 'geometry_msgs/msg/Twist'
    });

    function rotateToTargetOrientation() {
        const currentW = data.currentPosition.w;
        const error = targetOrientationW - currentW;

        if (Math.abs(error) > 0.02) {
            const twist = new ROSLIB.Message({
                linear: {x: 0, y: 0, z: 0},
                angular: {x: 0, y: 0, z: error > 0 ? 0.8 : -0.8}
            });
            cmdVelTopic.publish(twist);
            setTimeout(rotateToTargetOrientation, 100);
        } else {
            stop();
            moveToTarget();
        }
    }

    function moveToTarget() {
        moveIntervalId = setInterval(() => {
            const currX = data.currentPosition.x;
            const currY = data.currentPosition.y;
            const dist = Math.sqrt(Math.pow(targetX - currX, 2) + Math.pow(targetY - currY, 2));
    
            if (dist > 0.7) {
                const twist = new ROSLIB.Message({
                    linear: {x: 0.5, y: 0, z: 0},
                    angular: {x: 0, y: 0, z: 0}
                });
                cmdVelTopic.publish(twist);
            } else {
                clearInterval(moveIntervalId);
                moveIntervalId = null;
                stop();
            }
        }, 1000);
    }

    rotateToTargetOrientation();
}


function connect() {
    console.log("Clic en connect");

    updateRosBridgeAddress();

    data.ros = new ROSLIB.Ros({
        url: data.rosbridge_address
    });

    data.ros.on("connection", () => {
        data.connected = true;
        document.getElementById("estado").textContent = '🔌 Conectado';
        document.getElementById("estado").style.color = 'green';
        console.log("Conexión con ROSBridge correcta");

        // 准备地图画布  Preparación del lienzo del mapa
        const canvas = document.getElementById("mapCanvas");
        const ctx = canvas.getContext("2d");
        const image = new Image();
        let mapInfo = null;
        let robotPosition = { x: 0, y: 0 };

        // 处理图片加载成功  Procesando la carga de la imagen exitosamente
        image.onload = () => {
            console.log("Imagen cargada correctamente");
            canvas.width = image.width;
            canvas.height = image.height;
            ctx.drawImage(image, 0, 0);
        };

        image.onerror = () => {
            console.error("Error al cargar la imagen:", image.src);
        };

        // 加载 YAML 并设置图片路径 Cargar YAML y establecer la ruta de la imagen
        fetch(`http://${localIp}:8000/static/farmaciaMapa.yaml`)

            .then(res => res.text())
            .then(text => {
                mapInfo = jsyaml.load(text);
                console.log("🟡 YAML recibido:", mapInfo);
                image.src = `http://${localIp}:8000/static/` + mapInfo.image;

            });

        // 订阅 /odom：更新位置 & 地图画图 Suscríbete a /odom: Actualizar ubicación y dibujo del mapa
        const odomTopic = new ROSLIB.Topic({
            ros: data.ros,
            name: '/odom',
            messageType: 'nav_msgs/msg/Odometry'
        });

        odomTopic.subscribe((message) => {
            const position = message.pose.pose.position;
            const orientation = message.pose.pose.orientation;

            data.currentPosition = {
                x: position.x,
                y: position.y,
                z: position.z,
                w: orientation.w
            };

            document.getElementById("pos_x").textContent = position.x.toFixed(2);
            document.getElementById("pos_y").textContent = position.y.toFixed(2);
            document.getElementById("pos_w").textContent = orientation.w.toFixed(2);

            if (!mapInfo || !image.complete) return;

            robotPosition.x = position.x;
            robotPosition.y = position.y;

            const res = mapInfo.resolution;
            const origin = mapInfo.origin;
            const pixelX = (robotPosition.x - origin[0]) / res;
            const pixelY = canvas.height - ((robotPosition.y - origin[1]) / res);

            ctx.clearRect(0, 0, canvas.width, canvas.height);
            ctx.drawImage(image, 0, 0);

            ctx.beginPath();
            ctx.fillStyle = "green";
            ctx.arc(pixelX, pixelY, 5, 0, 2 * Math.PI);
            ctx.fill();
        });


        const temperaturaSpan = document.getElementById("temperatura");
    if (temperaturaSpan) {
        const tempTopic = new ROSLIB.Topic({
            ros: data.ros,
            name: '/temperature',
            messageType: 'sensor_msgs/msg/Temperature'
        });

        tempTopic.subscribe(function (message) {
            console.log("收到温度数据：", message.temperature);
            temperaturaSpan.textContent = `🌡 Temperatura: ${message.temperature.toFixed(1)} °C`;
        });
    } else {
        console.warn("⚠️ 没找到 temperatura 元素");
    }


        
    });

    data.ros.on("error", (error) => {
        console.error("Error de conexión:", error);
        document.getElementById("estado").textContent = 'Error de conexión';
        document.getElementById("estado").style.color = 'red';
    });

    data.ros.on("close", () => {
        data.connected = false;
        document.getElementById("estado").textContent = 'Desconectado';
        document.getElementById("estado").style.color = 'red';
        console.log("Conexión cerrada");
    });
}




function disconnect() {
    if (data.ros) {
        data.ros.close();
        data.connected = false;
        document.getElementById("estado").textContent = '🔌 Desconectado';
        document.getElementById("estado").style.color = 'red';
        console.log("Clic en desconectar");
    }
}

function move() {
    if (!data.connected) {
        console.warn("No conectado, no se puede mover");
        return;
    }

    const cmdVelTopic = new ROSLIB.Topic({
        ros: data.ros,
        name: '/cmd_vel',
        messageType: 'geometry_msgs/msg/Twist'
    });

    const message = new ROSLIB.Message({
        linear: {x: 0.1, y: 0, z: 0},
        angular: {x: 0, y: 0, z: (data.reverse ? 0.2 : -0.2)}
    });

    console.log("Publicando movimiento:", message);
    cmdVelTopic.publish(message);
}

function stop() {
    if (!data.connected) return;

    const cmdVelTopic = new ROSLIB.Topic({
        ros: data.ros,
        name: '/cmd_vel',
        messageType: 'geometry_msgs/msg/Twist'
    });

    const message = new ROSLIB.Message({
        linear: {x: 0, y: 0, z: 0},
        angular: {x: 0, y: 0, z: 0}
    });

    console.log("Deteniendo robot");
    cmdVelTopic.publish(message);
}

function moveRobot(direccion) {
    if (!data.connected) {
        console.warn("No conectado, no se puede mover");
        return;
    }

    const cmdVelTopic = new ROSLIB.Topic({
        ros: data.ros,
        name: '/cmd_vel',
        messageType: 'geometry_msgs/msg/Twist'
    });

    const message = new ROSLIB.Message({
        linear: {x: 0, y: 0, z: 0},
        angular: {x: 0, y: 0, z: 0}
    });

    switch (direccion) {
        case "delante":
            message.linear.x = 0.1;
            break;
        case "atras":
            message.linear.x = -0.1;
            break;
        case "izquierda":
            message.angular.z = 0.5;
            break;
        case "derecha":
            message.angular.z = -0.5;
            break;
    }

    console.log(`Moviendo ${direccion}:`, message);
    cmdVelTopic.publish(message);
}

function reverse() {
    data.reverse = !data.reverse;
    console.log("Dirección cambiada:", data.reverse ? "Adelante" : "Atrás");
}

function moveCasaToColaClientes() {
    if (!data.connected) {
        console.warn("No conectado, no se puede mover");
        return;
    }

    const { x, y, z, w } = data.currentPosition;

    console.log(`Coordenadas actuales: X=${x.toFixed(2)}, Y=${y.toFixed(2)}, Z=${z.toFixed(2)}, W=${w.toFixed(2)}`);

    const targetOrientationW = - 0.70;
    const targetX = 4.5;
    const targetY = 7.5;

    const cmdVelTopic = new ROSLIB.Topic({
        ros: data.ros,
        name: '/cmd_vel',
        messageType: 'geometry_msgs/msg/Twist'
    });

    // Función para girar hasta alcanzar orientación deseada
    function rotateToTargetOrientation() {
        const currentW = data.currentPosition.w;
        const error = targetOrientationW - currentW;

        if (Math.abs(error) > 0.01) { // tolerancia de 0.01
            const twist = new ROSLIB.Message({
                linear: {x: 0, y: 0, z: 0},
                angular: {x: 0, y: 0, z: error > 0 ? 0.8 : -0.8} // Aumenté la velocidad angular a 0.8
            });
            cmdVelTopic.publish(twist);

            // Volver a comprobar tras un pequeño retardo
            setTimeout(rotateToTargetOrientation, 100);
        } else {
            console.log("Orientación correcta alcanzada. Deteniendo giro...");
            stop();
            // Una vez orientado, avanzar hacia destino
            moveToTarget();
        }
    }

    // Función para moverse hacia las coordenadas destino
    function moveToTarget() {
        moveIntervalId = setInterval(() => {
            const currX = data.currentPosition.x;
            const currY = data.currentPosition.y;
    
            const dist = Math.sqrt(Math.pow(targetX - currX, 2) + Math.pow(targetY - currY, 2));
    
            // Añadir condición para evitar que el robot se pase de la coordenada en X
            if (dist > 0.7) { // tolerancia de 30 cm
                const twist = new ROSLIB.Message({
                    linear: {x: 0.5, y: 0, z: 0},
                    angular: {x: 0, y: 0, z: 0}
                });
    
                if (targetY > currY) {
                    cmdVelTopic.publish(twist);
                } else if (targetY < currY) {
                    const reverseTwist = new ROSLIB.Message({
                        linear: {x: -0.3, y: 0, z: 0},
                        angular: {x: 0, y: 0, z: 0}
                    });
                    cmdVelTopic.publish(reverseTwist);
                }
            } else {
                console.log("Destino alcanzado. Deteniendo robot...");
                clearInterval(moveIntervalId);
                moveIntervalId = null;
                stop();
            }
        }, 1000);
    }

    // Iniciar primer paso: rotar
    rotateToTargetOrientation();
}
function moveCasaToAlmacen() {
    if (!data.connected) {
        console.warn("No conectado, no se puede mover");
        return;
    }

    const { x, y, z, w } = data.currentPosition;

    console.log(`Coordenadas actuales: X=${x.toFixed(2)}, Y=${y.toFixed(2)}, Z=${z.toFixed(2)}, W=${w.toFixed(2)}`);

    const targetOrientationW = 0.48;
    const targetX = 2.76;
    const targetY = - 10.76;

    const cmdVelTopic = new ROSLIB.Topic({
        ros: data.ros,
        name: '/cmd_vel',
        messageType: 'geometry_msgs/msg/Twist'
    });

    // Función para girar hasta alcanzar orientación deseada
    function rotateToTargetOrientation() {
        const currentW = data.currentPosition.w;
        const error = targetOrientationW - currentW;

        if (Math.abs(error) > 0.02) { // tolerancia de 0.02
            const twist = new ROSLIB.Message({
                linear: {x: 0, y: 0, z: 0},
                angular: {x: 0, y: 0, z: error > 0 ? 0.8 : -0.8} // Aumenté la velocidad angular a 0.8
            });
            cmdVelTopic.publish(twist);

            // Volver a comprobar tras un pequeño retardo
            setTimeout(rotateToTargetOrientation, 100);
        } else {
            console.log("Orientación correcta alcanzada. Deteniendo giro...");
            stop();
            // Una vez orientado, avanzar hacia destino
            moveToTarget();
        }
    }

    // Función para moverse hacia las coordenadas destino
    function moveToTarget() {
        moveIntervalId = setInterval(() => {
            const currX = data.currentPosition.x;
            const currY = data.currentPosition.y;
    
            const dist = Math.sqrt(Math.pow(targetX - currX, 2) + Math.pow(targetY - currY, 2));
    
            if (dist > 0.7) {
                const twist = new ROSLIB.Message({
                    linear: {x: -0.5, y: 0, z: 0},
                    angular: {x: 0, y: 0, z: 0}
                });
    
                if (targetY > currY) {
                    cmdVelTopic.publish(twist);
                } else if (targetY < currY) {
                    const reverseTwist = new ROSLIB.Message({
                        linear: {x: 0.3, y: 0, z: 0},
                        angular: {x: 0, y: 0, z: 0}
                    });
                    cmdVelTopic.publish(reverseTwist);
                }
            } else {
                console.log("Destino alcanzado. Deteniendo robot...");
                clearInterval(moveIntervalId);
                moveIntervalId = null;
                stop();
            }
        }, 1000);
    }

    // Iniciar primer paso: rotar
    rotateToTargetOrientation();
} 
function moveColaClientesToAlmacen() {
    if (!data.connected) {
        console.warn("No conectado, no se puede mover");
        return;
    }

    const { x, y, z, w } = data.currentPosition;

    console.log(`Coordenadas actuales: X=${x.toFixed(2)}, Y=${y.toFixed(2)}, Z=${z.toFixed(2)}, W=${w.toFixed(2)}`);

    const targetOrientationW = 0.70;
    const targetX = 4.5;
    const targetY = - 8;

    const cmdVelTopic = new ROSLIB.Topic({
        ros: data.ros,
        name: '/cmd_vel',
        messageType: 'geometry_msgs/msg/Twist'
    });

    // Función para girar hasta alcanzar orientación deseada
    function rotateToTargetOrientation() {
        const currentW = data.currentPosition.w;
        const error = targetOrientationW - currentW;

        if (Math.abs(error) > 0.02) { // tolerancia de 0.02
            const twist = new ROSLIB.Message({
                linear: {x: 0, y: 0, z: 0},
                angular: {x: 0, y: 0, z: error > 0 ? 0.8 : -0.8} // Aumenté la velocidad angular a 0.8
            });
            cmdVelTopic.publish(twist);

            // Volver a comprobar tras un pequeño retardo
            setTimeout(rotateToTargetOrientation, 100);
        } else {
            console.log("Orientación correcta alcanzada. Deteniendo giro...");
            stop();
            // Una vez orientado, avanzar hacia destino
            moveToTarget();
        }
    }

    // Función para moverse hacia las coordenadas destino
    function moveToTarget() {
        moveIntervalId = setInterval(() => {
            const currX = data.currentPosition.x;
            const currY = data.currentPosition.y;
    
            const dist = Math.sqrt(Math.pow(targetX - currX, 2) + Math.pow(targetY - currY, 2));
    
            if (dist > 0.7) {
                const twist = new ROSLIB.Message({
                    linear: {x: -0.5, y: 0, z: 0},
                    angular: {x: 0, y: 0, z: 0}
                });
    
                if (targetY > currY) {
                    cmdVelTopic.publish(twist);
                } else if (targetY < currY) {
                    const reverseTwist = new ROSLIB.Message({
                        linear: {x: 0.3, y: 0, z: 0},
                        angular: {x: 0, y: 0, z: 0}
                    });
                    cmdVelTopic.publish(reverseTwist);
                }
            } else {
                console.log("Destino alcanzado. Deteniendo robot...");
                clearInterval(moveIntervalId);
                moveIntervalId = null;
                moveCasaToAlmacen();  
                stop();
            }
        }, 1000);
    }

    // Iniciar primer paso: rotar
    rotateToTargetOrientation();
}
function moveColaClientesToCasa() {
    if (!data.connected) {
        console.warn("No conectado, no se puede mover");
        return;
    }

    const { x, y, z, w } = data.currentPosition;

    console.log(`Coordenadas actuales: X=${x.toFixed(2)}, Y=${y.toFixed(2)}, Z=${z.toFixed(2)}, W=${w.toFixed(2)}`);

    const targetOrientationW = 0.70;
    const targetX = 4.5;
    const targetY = - 8;

    const cmdVelTopic = new ROSLIB.Topic({
        ros: data.ros,
        name: '/cmd_vel',
        messageType: 'geometry_msgs/msg/Twist'
    });

    // Función para girar hasta alcanzar orientación deseada
    function rotateToTargetOrientation() {
        const currentW = data.currentPosition.w;
        const error = targetOrientationW - currentW;

        if (Math.abs(error) > 0.02) { // tolerancia de 0.02
            const twist = new ROSLIB.Message({
                linear: {x: 0, y: 0, z: 0},
                angular: {x: 0, y: 0, z: error > 0 ? 0.8 : -0.8} // Aumenté la velocidad angular a 0.8
            });
            cmdVelTopic.publish(twist);

            // Volver a comprobar tras un pequeño retardo
            setTimeout(rotateToTargetOrientation, 100);
        } else {
            console.log("Orientación correcta alcanzada. Deteniendo giro...");
            stop();
            // Una vez orientado, avanzar hacia destino
            moveToTarget();
        }
    }

    // Función para moverse hacia las coordenadas destino
    function moveToTarget() {
        moveIntervalId = setInterval(() => {
            const currX = data.currentPosition.x;
            const currY = data.currentPosition.y;
    
            const dist = Math.sqrt(Math.pow(targetX - currX, 2) + Math.pow(targetY - currY, 2));
    
            if (dist > 0.7) {
                const twist = new ROSLIB.Message({
                    linear: {x: -0.5, y: 0, z: 0},
                    angular: {x: 0, y: 0, z: 0}
                });
    
                if (targetY > currY) {
                    cmdVelTopic.publish(twist);
                } else if (targetY < currY) {
                    const reverseTwist = new ROSLIB.Message({
                        linear: {x: 0.3, y: 0, z: 0},
                        angular: {x: 0, y: 0, z: 0}
                    });
                    cmdVelTopic.publish(reverseTwist);
                }
            } else {
                console.log("Destino alcanzado. Deteniendo robot...");
                clearInterval(moveIntervalId);
                moveIntervalId = null;
                stop();
            }
        }, 1000);
    }
    // Iniciar primer paso: rotar
    rotateToTargetOrientation();
}   
function moveAlmacenToColaClientes() {
    if (!data.connected) {
        console.warn("No conectado, no se puede mover");
        return;
    }

    const { x, y, z, w } = data.currentPosition;

    console.log(`Coordenadas actuales: X=${x.toFixed(2)}, Y=${y.toFixed(2)}, Z=${z.toFixed(2)}, W=${w.toFixed(2)}`);

    const targetOrientationW = - 0.94;
    const targetX = 4;
    const targetY = - 8.5;

    const cmdVelTopic = new ROSLIB.Topic({
        ros: data.ros,
        name: '/cmd_vel',
        messageType: 'geometry_msgs/msg/Twist'
    });

    // Función para girar hasta alcanzar orientación deseada
    function rotateToTargetOrientation() {
        const currentW = data.currentPosition.w;
        const error = targetOrientationW - currentW;

        if (Math.abs(error) > 0.02) { // tolerancia de 0.02
            const twist = new ROSLIB.Message({
                linear: {x: 0, y: 0, z: 0},
                angular: {x: 0, y: 0, z: -0.8} // Aumenté la velocidad angular a 0.8
            });
            cmdVelTopic.publish(twist);

            // Volver a comprobar tras un pequeño retardo
            setTimeout(rotateToTargetOrientation, 100);
        } else {
            console.log("Orientación correcta alcanzada. Deteniendo giro...");
            stop();
            // Una vez orientado, avanzar hacia destino
            moveToTarget();
        }
    }

    // Función para moverse hacia las coordenadas destino
    function moveToTarget() {
        moveIntervalId = setInterval(() => {
            const currX = data.currentPosition.x;
            const currY = data.currentPosition.y;
    
            const dist = Math.sqrt(Math.pow(targetX - currX, 2) + Math.pow(targetY - currY, 2));
    
            if (dist > 0.7) {
                const twist = new ROSLIB.Message({
                    linear: {x: 0.5, y: 0, z: 0},
                    angular: {x: 0, y: 0, z: 0}
                });
    
                if (targetY > currY) {
                    cmdVelTopic.publish(twist);
                } else if (targetY < currY) {
                    const reverseTwist = new ROSLIB.Message({
                        linear: {x: -0.3, y: 0, z: 0},
                        angular: {x: 0, y: 0, z: 0}
                    });
                    cmdVelTopic.publish(reverseTwist);
                }
            } else {
                console.log("Destino alcanzado. Deteniendo robot...");
                clearInterval(moveIntervalId);
                moveIntervalId = null;
                moveCasaToColaClientes();  
                stop();
            }
        }, 1000);
    }
    // Iniciar primer paso: rotar
    rotateToTargetOrientation();



}
function moveAlmacenToCasa() {
    if (!data.connected) {
        console.warn("No conectado, no se puede mover");
        return;
    }

    const { x, y, z, w } = data.currentPosition;

    console.log(`Coordenadas actuales: X=${x.toFixed(2)}, Y=${y.toFixed(2)}, Z=${z.toFixed(2)}, W=${w.toFixed(2)}`);

    const targetOrientationW = - 0.94;
    const targetX = 4.5;
    const targetY = - 8;

    const cmdVelTopic = new ROSLIB.Topic({
        ros: data.ros,
        name: '/cmd_vel',
        messageType: 'geometry_msgs/msg/Twist'
    });

    // Función para girar hasta alcanzar orientación deseada
    function rotateToTargetOrientation() {
        const currentW = data.currentPosition.w;
        const error = targetOrientationW - currentW;

        if (Math.abs(error) > 0.02) { // tolerancia de 0.02
            const twist = new ROSLIB.Message({
                linear: {x: 0, y: 0, z: 0},
                angular: {x: 0, y: 0, z: -0.8} // Aumenté la velocidad angular a 0.8
            });
            cmdVelTopic.publish(twist);

            // Volver a comprobar tras un pequeño retardo
            setTimeout(rotateToTargetOrientation, 100);
        } else {
            console.log("Orientación correcta alcanzada. Deteniendo giro...");
            stop();
            // Una vez orientado, avanzar hacia destino
            moveToTarget();
        }
    }

    // Función para moverse hacia las coordenadas destino
    function moveToTarget() {
        moveIntervalId = setInterval(() => {
            const currX = data.currentPosition.x;
            const currY = data.currentPosition.y;
    
            const dist = Math.sqrt(Math.pow(targetX - currX, 2) + Math.pow(targetY - currY, 2));
    
            if (dist > 0.7) {
                const twist = new ROSLIB.Message({
                    linear: {x: 0.5, y: 0, z: 0},
                    angular: {x: 0, y: 0, z: 0}
                });
    
                if (targetY > currY) {
                    cmdVelTopic.publish(twist);
                } else if (targetY < currY) {
                    const reverseTwist = new ROSLIB.Message({
                        linear: {x: -0.3, y: 0, z: 0},
                        angular: {x: 0, y: 0, z: 0}
                    });
                    cmdVelTopic.publish(reverseTwist);
                }
            } else {
                console.log("Destino alcanzado. Deteniendo robot...");
                clearInterval(moveIntervalId);
                moveIntervalId = null;
                stop();
            }
        }, 1000);
    }

    // Iniciar primer paso: rotar
    rotateToTargetOrientation();
} 


function updateCameraFeed() {
    const img = document.getElementById("cameraFeed");
    const timestamp = new Date().getTime(); // Evitar el almacenamiento en caché
    img.src = `http://${localIp}:8080/stream?topic=/image&timestamp=${timestamp}`;
  }
  
  
  // 每隔 2 秒刷新一次图像--Actualizar la imagen cada 2 segundos
  setInterval(updateCameraFeed, 2000);

document.addEventListener('DOMContentLoaded', event => {
    
    // 自动根据页面地址设置 IP     Establecer IP automáticamente según la dirección de la página
    document.getElementById("ipInput").value = `ws://${localIp}:9090`;
    const timestamp = new Date().getTime(); 
    document.getElementById("cameraFeed").src = `http://${localIp}:8080/stream?topic=/image&timestamp=${timestamp}`;
    



    document.getElementById("btn_goto_cola").addEventListener("click", () => {
        sendNavGoal(3.998915, 4.900286, 1.0);
    });
    document.getElementById("btn_goto_casa").addEventListener("click", () => {
        sendNavGoal(4.500114, -8.000002, 1.0);
    });
    document.getElementById("btn_goto_almacen").addEventListener("click", () => {
        sendNavGoal(0.009239, -15.876596, 1.0);
    });
    

    console.log("Entro en la página");

    // Botones principales
    document.getElementById("btn_con").addEventListener("click", connect);
    document.getElementById("btn_dis").addEventListener("click", disconnect);
    document.getElementById("btn_move").addEventListener("click", move);
    document.getElementById("btn_stop").addEventListener("click", stop);
    document.getElementById("btn_reverse").addEventListener("click", reverse);


    // Botones de movimiento
    document.getElementById("btn_wsad_delante").addEventListener("click", () => moveRobot("delante"));
    document.getElementById("btn_wsad_atras").addEventListener("click", () => moveRobot("atras"));
    document.getElementById("btn_wsad_izquierda").addEventListener("click", () => moveRobot("izquierda"));
    document.getElementById("btn_wsad_derecha").addEventListener("click", () => moveRobot("derecha"));
    document.getElementById("btn_wsad_parar").addEventListener("click", stop);
    document.getElementById("btn_cancelar_nav").addEventListener("click", cancelNavigation);

    //  Suscribirse al topic /ultimo_sip para actualizar el SIP del último cliente
    if (data.ros) {
        const ultimoSipTopic = new ROSLIB.Topic({
            ros: data.ros,
            name: '/ultimo_sip',
            messageType: 'std_msgs/msg/String'
        });

        ultimoSipTopic.subscribe(function(message) {
            const sipSpan = document.getElementById("sip-ultimo-cliente");
            if (sipSpan) {
                sipSpan.textContent = message.data;
            }
        });
    }
        
});


document.addEventListener("DOMContentLoaded", function () {
    const btnGuardar = document.getElementById("btn_guardar_imagen");
    const cameraFeed = document.getElementById("cameraFeed");
    const sipSpan = document.getElementById("sip-ultimo-cliente");

    console.log("✅ Página cargada, preparando botón de Escanear SIP");

    if (!btnGuardar) {
        console.error("❌ No se encontró el botón btn_guardar_imagen");
        return;
    }

    if (!cameraFeed) {
        console.error("❌ No se encontró el elemento cameraFeed");
        return;
    }

    // Al hacer clic en el botón, capturamos la imagen desde canvas
    btnGuardar.addEventListener("click", function () {
        console.log("📸 Botón pulsado para capturar imagen");

        // Crear canvas temporal
        const canvas = document.createElement("canvas");
        canvas.width = cameraFeed.videoWidth || cameraFeed.naturalWidth;
        canvas.height = cameraFeed.videoHeight || cameraFeed.naturalHeight;

        const ctx = canvas.getContext("2d");
        ctx.drawImage(cameraFeed, 0, 0, canvas.width, canvas.height);

        // Convertir a blob y descargar
        canvas.toBlob(function (blob) {
            if (blob) {
                const url = URL.createObjectURL(blob);
                const a = document.createElement("a");
                a.href = url;
                a.download = "robot_camera.jpg";
                document.body.appendChild(a);
                a.click();
                document.body.removeChild(a);
                URL.revokeObjectURL(url);
                console.log("✅ Imagen descargada correctamente");
            } else {
                alert("⚠️ No se pudo capturar la imagen.");
            }
        }, "image/jpeg");
    });

    // Suscripción al topic /ultimo_sip (si es necesario)
    if (typeof data !== 'undefined' && data.ros) {
        const ultimoSipTopic = new ROSLIB.Topic({
            ros: data.ros,
            name: '/ultimo_sip',
            messageType: 'std_msgs/msg/String'
        });

        ultimoSipTopic.subscribe(function (message) {
            if (sipSpan) {
                sipSpan.textContent = message.data;
                console.log("🟢 Recibido desde /ultimo_sip: " + message.data);
            }
        });
    }
});






