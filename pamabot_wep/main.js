document.addEventListener('DOMContentLoaded', event => {
    console.log("entro en la página");

    document.getElementById("btn_con").addEventListener("click", connect);
    document.getElementById("btn_dis").addEventListener("click", disconnect);
    document.getElementById("btn_move").addEventListener("click", move);
    document.getElementById("btn_stop").addEventListener("click", stop);
    document.getElementById("btn_reverse").addEventListener("click", reverse);

    let data = {
        // Conexión ROS
        ros: null,
        rosbridge_address: 'ws://10.0.2.15:9090/',  // Dirección por defecto
        connected: false,
        // Variable para controlar el estado de dirección
        reverse: false
    };

    // Actualiza la dirección del servidor según el input
    function updateRosBridgeAddress() {
        const ipInput = document.getElementById("ipInput").value;
        data.rosbridge_address = ipInput;
    }

    function connect() {
        console.log("Clic en connect");

        // Actualiza la dirección del servidor antes de intentar la conexión
        updateRosBridgeAddress();

        data.ros = new ROSLIB.Ros({
            url: data.rosbridge_address
        });

        // Define callbacks
        data.ros.on("connection", () => {
            data.connected = true;
            document.getElementById("estado").textContent = '🔌 Conectado';
            document.getElementById("estado").style.color = 'green';
            console.log("Conexión con ROSBridge correcta");

            // Suscribirse al topic '/odom' para recibir la posición
            const topic = new ROSLIB.Topic({
                ros: data.ros,
                name: '/odom',
                messageType: 'nav_msgs/msg/Odometry'
            });

            // Suscribirse a las actualizaciones del topic '/odom'
            topic.subscribe((message) => {
                const position = message.pose.pose.position;
                document.getElementById("pos_x").textContent = position.x.toFixed(2);
                document.getElementById("pos_y").textContent = position.y.toFixed(2);
            });
        });

        data.ros.on("error", (error) => {
            console.log("Se ha producido un error en la conexión");
            console.log(error);
            document.getElementById("estado").textContent = '❌ Error de conexión';
            document.getElementById("estado").style.color = 'red';
        });

        data.ros.on("close", () => {
            data.connected = false;
            document.getElementById("estado").textContent = '🔌 Desconectado';
            document.getElementById("estado").style.color = 'red';
            console.log("Conexión con ROSBridge cerrada");
        });
    }

    function disconnect() {
        if (data.ros) {
            data.ros.close();
            data.connected = false;
            document.getElementById("estado").textContent = '🔌 Desconectado';
            document.getElementById("estado").style.color = 'red';
            console.log('Clic en botón de desconexión');
        }
    }

    function move() {
        if (!data.connected) {
            console.log("No conectado, no se puede mover");
            return;
        }

        const topic = new ROSLIB.Topic({
            ros: data.ros,
            name: '/cmd_vel',
            messageType: 'geometry_msgs/msg/Twist'
        });

        // Depuración para ver el valor de 'reverse'
        console.log("Dirección actual (reverse):", data.reverse ? "Adelante" : "Atrás");

        // Publicamos el mensaje con dirección actual
        let message = new ROSLIB.Message({
            linear: {x: 0.1, y: 0, z: 0},
            angular: {x: 0, y: 0, z: (data.reverse ? 0.2 : -0.2)} // Cambia el signo aquí según la variable reverse
        });

        console.log("Publicando mensaje de movimiento:", message);
        topic.publish(message);
    }

    function stop() {
        if (!data.connected) return;

        const topic = new ROSLIB.Topic({
            ros: data.ros,
            name: '/cmd_vel',
            messageType: 'geometry_msgs/msg/Twist'
        });

        let message = new ROSLIB.Message({
            linear: {x: 0, y: 0, z: 0},
            angular: {x: 0, y: 0, z: 0}
        });

        console.log("Deteniendo el robot");
        topic.publish(message);
    }

    function reverse() {
        // Cambiar el valor de la dirección
        data.reverse = !data.reverse;
        console.log("Dirección cambiada:", data.reverse ? "Adelante" : "Atrás");
    }
});
