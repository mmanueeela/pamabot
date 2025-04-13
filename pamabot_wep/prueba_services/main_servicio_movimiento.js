document.addEventListener('DOMContentLoaded', event => {
    console.log("Entro en la página");

    // Botones principales
    document.getElementById("btn_con").addEventListener("click", connect);
    document.getElementById("btn_dis").addEventListener("click", disconnect);
    document.getElementById("btn_move").addEventListener("click", move);
    document.getElementById("btn_stop").addEventListener("click", stop);
    document.getElementById("btn_reverse").addEventListener("click", reverse);
    //Botones de movimiento a coordenadas
    document.getElementById("btn_move_casa_to_cola_clientes").addEventListener("click", moveCasaToColaClientes);
    document.getElementById("btn_move_cola_to_almacen").addEventListener("click", moveColaClientesToAlmacen);
    document.getElementById("btn_move_almacen_to_cola").addEventListener("click", moveAlmacenToColaClientes);
    document.getElementById("btn_move_cola_to_casa").addEventListener("click", moveColaClientesToCasa);
    document.getElementById("btn_move_casa_to_almacen").addEventListener("click", moveCasaToAlmacen);
    document.getElementById("btn_move_almacen_to_casa").addEventListener("click", moveAlmacenToCasa);


    // Botones de movimiento
    document.getElementById("btn_delante").addEventListener("click", () => moveRobot("delante"));
    document.getElementById("btn_atras").addEventListener("click", () => moveRobot("atras"));
    document.getElementById("btn_izquierda").addEventListener("click", () => moveRobot("izquierda"));
    document.getElementById("btn_derecha").addEventListener("click", () => moveRobot("derecha"));
    document.getElementById("btn_parar").addEventListener("click", stop);

    let data = {
        ros: null,
        rosbridge_address: 'ws://10.0.2.15:9090/',
        connected: false,
        reverse: false,
        currentPosition: { x: 0, y: 0, z: 0, w: 1 } // Inicializar posición actual
    };

    function updateRosBridgeAddress() {
        const ipInput = document.getElementById("ipInput").value;
        data.rosbridge_address = ipInput || data.rosbridge_address;
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

            // Suscribirse a /odom
            const odomTopic = new ROSLIB.Topic({
                ros: data.ros,
                name: '/odom',
                messageType: 'nav_msgs/msg/Odometry'
            });

            odomTopic.subscribe((message) => {
                const position = message.pose.pose.position;
                const orientation = message.pose.pose.orientation;

                data.currentPosition.x = position.x;
                data.currentPosition.y = position.y;
                data.currentPosition.z = position.z;
                data.currentPosition.w = orientation.w;

                document.getElementById("pos_x").textContent = position.x.toFixed(2);
                document.getElementById("pos_y").textContent = position.y.toFixed(2);
                document.getElementById("pos_w").textContent = orientation.w.toFixed(2); // CORRECTO: orientation.w
            });
        });

        data.ros.on("error", (error) => {
            console.error("Error de conexión:", error);
            document.getElementById("estado").textContent = '❌ Error de conexión';
            document.getElementById("estado").style.color = 'red';
        });

        data.ros.on("close", () => {
            data.connected = false;
            document.getElementById("estado").textContent = '🔌 Desconectado';
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
            const moveInterval = setInterval(() => {
                const currX = data.currentPosition.x;
                const currY = data.currentPosition.y;
        
                const dist = Math.sqrt(Math.pow(targetX - currX, 2) + Math.pow(targetY - currY, 2));
        
                // Añadir condición para evitar que el robot se pase de la coordenada en X
                if (dist > 0.7) { // tolerancia de 30 cm
                    const twist = new ROSLIB.Message({
                        linear: {x: 0.5, y: 0, z: 0}, // Ajustar la velocidad lineal para no pasar de la coordenada
                        angular: {x: 0, y: 0, z: 0}
                    });
        
                    // Asegurarse de que no pase la coordenada de destino en X
                    if (targetY > currY) {
                        cmdVelTopic.publish(twist);
                    } else if (targetY < currY) {
                        const reverseTwist = new ROSLIB.Message({
                            linear: {x: -0.3, y: 0, z: 0}, // Mover hacia atrás si se pasa del objetivo
                            angular: {x: 0, y: 0, z: 0}
                        });
                        cmdVelTopic.publish(reverseTwist);
                    }
                } else {
                    console.log("Destino alcanzado. Deteniendo robot...");
                    clearInterval(moveInterval);
                    stop();
                }
            }, 1000); // Comprobar cada 1000 ms
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
            const moveInterval = setInterval(() => {
                const currX = data.currentPosition.x;
                const currY = data.currentPosition.y;
        
                const dist = Math.sqrt(Math.pow(targetX - currX, 2) + Math.pow(targetY - currY, 2));
        
                // Añadir condición para evitar que el robot se pase de la coordenada en X
                if (dist > 0.7) { // tolerancia de 30 cm
                    const twist = new ROSLIB.Message({
                        linear: {x: -0.5, y: 0, z: 0}, // Ajustar la velocidad lineal para no pasar de la coordenada
                        angular: {x: 0, y: 0, z: 0}
                    });
        
                    // Asegurarse de que no pase la coordenada de destino en X
                    if (targetY > currY) {
                        cmdVelTopic.publish(twist);
                    } else if (targetY < currY) {
                        const reverseTwist = new ROSLIB.Message({
                            linear: {x: 0.3, y: 0, z: 0}, // Mover hacia atrás si se pasa del objetivo
                            angular: {x: 0, y: 0, z: 0}
                        });
                        cmdVelTopic.publish(reverseTwist);
                    }
                } else {
                    console.log("Destino alcanzado. Deteniendo robot...");
                    clearInterval(moveInterval);
                    stop();
                }
            }, 1000); // Comprobar cada 1000 ms
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
            const moveInterval = setInterval(() => {
                const currX = data.currentPosition.x;
                const currY = data.currentPosition.y;
        
                const dist = Math.sqrt(Math.pow(targetX - currX, 2) + Math.pow(targetY - currY, 2));
        
                // Añadir condición para evitar que el robot se pase de la coordenada en X
                if (dist > 0.7) { // tolerancia de 30 cm
                    const twist = new ROSLIB.Message({
                        linear: {x: -0.5, y: 0, z: 0}, // Ajustar la velocidad lineal para no pasar de la coordenada
                        angular: {x: 0, y: 0, z: 0}
                    });
        
                    // Asegurarse de que no pase la coordenada de destino en X
                    if (targetY > currY) {
                        cmdVelTopic.publish(twist);
                    } else if (targetY < currY) {
                        const reverseTwist = new ROSLIB.Message({
                            linear: {x: 0.3, y: 0, z: 0}, // Mover hacia atrás si se pasa del objetivo
                            angular: {x: 0, y: 0, z: 0}
                        });
                        cmdVelTopic.publish(reverseTwist);
                    }
                } else {
                    moveCasaToAlmacen();
                    clearInterval(moveInterval);
                    stop();
                }
            }, 1000); // Comprobar cada 1000 ms
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
            const moveInterval = setInterval(() => {
                const currX = data.currentPosition.x;
                const currY = data.currentPosition.y;
        
                const dist = Math.sqrt(Math.pow(targetX - currX, 2) + Math.pow(targetY - currY, 2));
        
                // Añadir condición para evitar que el robot se pase de la coordenada en X
                if (dist > 0.7) { // tolerancia de 30 cm
                    const twist = new ROSLIB.Message({
                        linear: {x: -0.5, y: 0, z: 0}, // Ajustar la velocidad lineal para no pasar de la coordenada
                        angular: {x: 0, y: 0, z: 0}
                    });
        
                    // Asegurarse de que no pase la coordenada de destino en X
                    if (targetY > currY) {
                        cmdVelTopic.publish(twist);
                    } else if (targetY < currY) {
                        const reverseTwist = new ROSLIB.Message({
                            linear: {x: 0.3, y: 0, z: 0}, // Mover hacia atrás si se pasa del objetivo
                            angular: {x: 0, y: 0, z: 0}
                        });
                        cmdVelTopic.publish(reverseTwist);
                    }
                } else {
                    console.log("Destino alcanzado. Deteniendo robot...");
                    clearInterval(moveInterval);
                    stop();
                }
            }, 1000); // Comprobar cada 1000 ms
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
            const moveInterval = setInterval(() => {
                const currX = data.currentPosition.x;
                const currY = data.currentPosition.y;
        
                const dist = Math.sqrt(Math.pow(targetX - currX, 2) + Math.pow(targetY - currY, 2));
        
                // Añadir condición para evitar que el robot se pase de la coordenada en X
                if (dist > 0.7) { // tolerancia de 30 cm
                    const twist = new ROSLIB.Message({
                        linear: {x: 0.5, y: 0, z: 0}, // Ajustar la velocidad lineal para no pasar de la coordenada
                        angular: {x: 0, y: 0, z: 0}
                    });
        
                    // Asegurarse de que no pase la coordenada de destino en X
                    if (targetY > currY) {
                        cmdVelTopic.publish(twist);
                    } else if (targetY < currY) {
                        const reverseTwist = new ROSLIB.Message({
                            linear: {x: -0.3, y: 0, z: 0}, // Mover hacia atrás si se pasa del objetivo
                            angular: {x: 0, y: 0, z: 0}
                        });
                        cmdVelTopic.publish(reverseTwist);
                    }
                } else {
                    moveCasaToColaClientes();
                    clearInterval(moveInterval);
                    stop();
                }
            }, 1000); // Comprobar cada 1000 ms
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
            const moveInterval = setInterval(() => {
                const currX = data.currentPosition.x;
                const currY = data.currentPosition.y;
        
                const dist = Math.sqrt(Math.pow(targetX - currX, 2) + Math.pow(targetY - currY, 2));
        
                // Añadir condición para evitar que el robot se pase de la coordenada en X
                if (dist > 0.7) { // tolerancia de 30 cm
                    const twist = new ROSLIB.Message({
                        linear: {x: 0.5, y: 0, z: 0}, // Ajustar la velocidad lineal para no pasar de la coordenada
                        angular: {x: 0, y: 0, z: 0}
                    });
        
                    // Asegurarse de que no pase la coordenada de destino en X
                    if (targetY > currY) {
                        cmdVelTopic.publish(twist);
                    } else if (targetY < currY) {
                        const reverseTwist = new ROSLIB.Message({
                            linear: {x: -0.3, y: 0, z: 0}, // Mover hacia atrás si se pasa del objetivo
                            angular: {x: 0, y: 0, z: 0}
                        });
                        cmdVelTopic.publish(reverseTwist);
                    }
                } else {
                    console.log("Destino alcanzado. Deteniendo robot...");
                    clearInterval(moveInterval);
                    stop();
                }
            }, 1000); // Comprobar cada 1000 ms
        }
    
        // Iniciar primer paso: rotar
        rotateToTargetOrientation();
    } 
});