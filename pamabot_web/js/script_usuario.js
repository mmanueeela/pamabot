document.addEventListener("DOMContentLoaded", () => {
  // --- Datos del paciente ---
  const editarDatosButton = document.querySelector('[data-bs-target="#editarDatosModal"]');
  const editarDatosPopup = document.getElementById("editarDatosPopup");
  const guardarDatosButton = document.getElementById("guardarDatos");
  const cancelarEdicionButton = document.getElementById("cancelarEdicion");

  const inputsCard = {
    nombre: document.getElementById("nombre"),
    apellido: document.getElementById("apellido"),
    sip: document.getElementById("sip"),
    correo: document.getElementById("correo"),
    telefono: document.getElementById("telefono"),
  };

  const inputsPopup = {
    nombre: document.getElementById("popupNombre"),
    apellido: document.getElementById("popupApellido"),
    sip: document.getElementById("popupSip"),
    correo: document.getElementById("popupCorreo"),
    telefono: document.getElementById("popupTelefono"),
  };

  Object.values(inputsCard).forEach(input => input.disabled = true);

  editarDatosButton.addEventListener("click", () => {
    Object.keys(inputsCard).forEach(key => {
      inputsPopup[key].value = inputsCard[key].value;
    });
    editarDatosPopup.style.display = "flex";
  });

  guardarDatosButton.addEventListener("click", () => {
    Object.keys(inputsCard).forEach(key => {
      inputsCard[key].value = inputsPopup[key].value;
    });
    alert("Datos guardados correctamente.");
    editarDatosPopup.style.display = "none";
  });

  cancelarEdicionButton.addEventListener("click", () => {
    editarDatosPopup.style.display = "none";
  });

  // --- Notificaciones ---
  const notificacionesButton = document.querySelector('[data-bs-target="#notificacionesModal"]');
  const notificacionesPopup = document.getElementById("notificacionesPopup");
  const cerrarNotificacionesButton = document.getElementById("cerrarNotificaciones");
  const notificacionesLista = document.getElementById("notificacionesLista");

  const verificarNotificaciones = () => {
    if (notificacionesLista.querySelectorAll("li:not(.fija)").length === 0) {
      notificacionesLista.innerHTML += '<li class="sin-notificaciones">No hay notificaciones</li>';
    }
  };

  const actualizarPrimeraNotificacion = () => {
    const primeraNotificacion = notificacionesLista.querySelector("li span");
    if (!primeraNotificacion) return;

    const hoy = new Date();
    const recetas = [];

    document.querySelectorAll(".receta-tabla tbody tr").forEach(fila => {
      const celdas = fila.querySelectorAll("td");
      if (celdas.length < 9) return;
      const nombre = celdas[2].textContent.trim();
      const fechaStr = celdas[8].textContent.trim();
      const [d, m, a] = fechaStr.split("/").map(n => parseInt(n));
      const fecha = new Date(`20${a}`, m - 1, d);
      const dias = Math.ceil((fecha - hoy) / (1000 * 60 * 60 * 24));
      recetas.push({ nombre, dias });
    });

    recetas.sort((a, b) => a.dias - b.dias);
    const texto = recetas.map(r => `${r.nombre} vence en ${r.dias} días`).join("<br>");
    primeraNotificacion.innerHTML = `<strong>Resumen de recetas:</strong><br>${texto}`;

    const primerBoton = notificacionesLista.querySelector("li .redirigir_notificacion_receta");
    if (primerBoton) {
      primerBoton.onclick = () => {
        notificacionesPopup.style.display = "none";
        document.getElementById("recetasPopup").style.display = "flex";
      };
    }
  };

  const generarNotificacionesRecetas = () => {
    // ✅ Solo elimina las notificaciones que **NO** sean fijas
    notificacionesLista.querySelectorAll("li:not(.fija)").forEach(li => li.remove());

    const hoy = new Date();
    document.querySelectorAll(".receta-tabla tbody tr").forEach(fila => {
      const celdas = fila.querySelectorAll("td");
      if (celdas.length < 9) return;

      const nombre = celdas[2].textContent.trim();
      const fechaStr = celdas[8].textContent.trim();
      const [d, m, a] = fechaStr.split("/").map(Number);
      const fecha = new Date(`20${a}`, m - 1, d);
      const dias = Math.ceil((fecha - hoy) / (1000 * 60 * 60 * 24));

      if (dias <= 5 && dias >= 0) {
        const alerta = document.createElement("li");
        alerta.innerHTML = `
          <span style="color:${dias <= 1 ? 'red' : dias <= 3 ? 'orange' : 'goldenrod'}">
            ¡Atención! ${nombre} vence en ${dias} días
          </span>
          <div class="popup-buttons">
            <button class="btn boton eliminar-notificacion"><i class="bi bi-trash"></i></button>
            <button class="btn boton redirigir_notificacion_receta"><i class="bi bi-arrow-right-circle"></i></button>
          </div>
        `;
        notificacionesLista.appendChild(alerta);
      }
    });

    // Botones
    document.querySelectorAll(".eliminar-notificacion").forEach((btn) => {
      btn.onclick = (e) => {
        const li = e.target.closest("li");
        if (li && !li.classList.contains("fija")) li.remove();
        verificarNotificaciones();
      };
    });

    document.querySelectorAll(".redirigir_notificacion_receta").forEach((btn) => {
      btn.onclick = () => {
        notificacionesPopup.style.display = "none";
        document.getElementById("recetasPopup").style.display = "flex";
      };
    });
  };

  if (notificacionesButton) {
    notificacionesButton.addEventListener("click", () => {
      notificacionesPopup.style.display = "flex";
      actualizarPrimeraNotificacion();
      generarNotificacionesRecetas();
    });
  }

  if (cerrarNotificacionesButton) {
    cerrarNotificacionesButton.addEventListener("click", () => {
      notificacionesPopup.style.display = "none";
    });
  }

  document.querySelectorAll(".redirigir_notificacion_receta").forEach((btn) => {
    btn.addEventListener("click", () => {
      notificacionesPopup.style.display = "none";
      document.getElementById("recetasPopup").style.display = "flex";
    });
  });

  document.querySelectorAll(".redirigir_notificacion_compra").forEach((btn) => {
    btn.addEventListener("click", () => {
      notificacionesPopup.style.display = "none";
      document.getElementById("comprasPopup").style.display = "flex";
    });
  });

  // --- Recetas Popup ---
  const recetasButton = document.querySelector('[data-bs-target="#recetasModal"]');
  const recetasPopup = document.getElementById("recetasPopup");
  const cerrarRecetasButton = document.getElementById("cerrarRecetas");

  if (recetasButton) {
    recetasButton.addEventListener("click", () => {
      recetasPopup.style.display = "flex";
    });
  }

  if (cerrarRecetasButton) {
    cerrarRecetasButton.addEventListener("click", () => {
      recetasPopup.style.display = "none";
    });
  }

  // --- Historial ---
  const historialButton = document.querySelector('[data-bs-target="#historialModal"]');
  const comprasPopup = document.getElementById("comprasPopup");
  const cerrarHistorialButton = document.getElementById("cerrarHistorial");

  if (historialButton) {
    historialButton.addEventListener("click", () => {
      comprasPopup.style.display = "flex";
    });
  }

  if (cerrarHistorialButton) {
    cerrarHistorialButton.addEventListener("click", () => {
      comprasPopup.style.display = "none";
    });
  }

  // --- Edición de fecha ---
  document.querySelectorAll(".editar-fecha-btn").forEach((btn) => {
    btn.addEventListener("click", () => {
      const fila = btn.closest("tr");
      const celdas = fila.querySelectorAll("td");
      if (celdas.length < 9) return;

      const celdaFecha = celdas[8];
      const fechaActual = celdaFecha.textContent.trim();
      const nuevaFecha = prompt("Introduce nueva fecha (DD/MM/AA):", fechaActual);

      if (nuevaFecha && /^\d{2}\/\d{2}\/\d{2}$/.test(nuevaFecha)) {
        celdaFecha.textContent = nuevaFecha;
        alert("Fecha modificada: " + nuevaFecha);
        actualizarPrimeraNotificacion();
        generarNotificacionesRecetas();
      } else if (nuevaFecha) {
        alert("Formato incorrecto. Usa DD/MM/AA");
      }
    });
  });

  // Inicialización
  actualizarPrimeraNotificacion();
  generarNotificacionesRecetas();
});
