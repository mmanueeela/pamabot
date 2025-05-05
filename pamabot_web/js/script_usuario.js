document.addEventListener("DOMContentLoaded", () => {
  // Selección de elementos
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

  // Deshabilitar inputs al cargar la página
  Object.values(inputsCard).forEach((input) => {
    input.disabled = true;
  });

  // Abrir popup y copiar valores de la tarjeta
  editarDatosButton.addEventListener("click", () => {
    Object.keys(inputsCard).forEach((key) => {
      inputsPopup[key].value = inputsCard[key].value;
    });
    editarDatosPopup.style.display = "flex";
  });

  // Guardar cambios
  guardarDatosButton.addEventListener("click", () => {
    Object.keys(inputsCard).forEach((key) => {
      inputsCard[key].value = inputsPopup[key].value;
    });
    alert("Datos guardados correctamente.");
    editarDatosPopup.style.display = "none";
  });

  // Cancelar edición
  cancelarEdicionButton.addEventListener("click", () => {
    editarDatosPopup.style.display = "none";
  });

  const notificacionesButton = document.querySelector('[data-bs-target="#notificacionesModal"]');
  const notificacionesPopup = document.getElementById("notificacionesPopup");
  const cerrarNotificacionesButton = document.getElementById("cerrarNotificaciones");

  // Mostrar el popup de notificaciones
  if (notificacionesButton) {
    notificacionesButton.addEventListener("click", () => {
      notificacionesPopup.style.display = "flex";
    });
  }

  // Cerrar el popup de notificaciones
  if (cerrarNotificacionesButton) {
    cerrarNotificacionesButton.addEventListener("click", () => {
      notificacionesPopup.style.display = "none";
    });
  }

  const notificacionesLista = document.getElementById("notificacionesLista");

  // Función para verificar si hay notificaciones
  const verificarNotificaciones = () => {
    if (notificacionesLista.children.length === 0) {
      notificacionesLista.innerHTML = '<li class="sin-notificaciones">No hay notificaciones</li>';
    }
  };

  // Llamar a la función al cargar la página
  verificarNotificaciones();

  // Eliminar una notificación
  document.querySelectorAll(".eliminar-notificacion").forEach((button) => {
    button.addEventListener("click", (e) => {
      const notificacion = e.target.closest("li");
      if (notificacion) {
        notificacion.remove();
        verificarNotificaciones(); // Verificar si quedan notificaciones
      }
    });
  });

  // Redirigir al apartado Recetas
  document.querySelectorAll(".redirigir_notificacion_receta").forEach((button) => {
    button.addEventListener("click", () => {
      // Cerrar el popup de notificaciones
    const notificacionesPopup = document.getElementById("notificacionesPopup");
    notificacionesPopup.style.display = "none";

    // Abrir el popup de editar datos
    const recetasPopup = document.getElementById("recetasPopup");
    recetasPopup.style.display = "flex";
    });
  });
  
  // Redirigir al apartado Compras
  document.querySelectorAll(".redirigir_notificacion_compra").forEach((button) => {
    button.addEventListener("click", () => {
      // Cerrar el popup de notificaciones
    const notificacionesPopup = document.getElementById("notificacionesPopup");
    notificacionesPopup.style.display = "none";

    // Abrir el popup de editar datos
    const comprasPopup = document.getElementById("comprasPopup");
    comprasPopup.style.display = "flex";
    });
  });

  const recetasButton = document.querySelector('[data-bs-target="#recetasModal"]');
  const recetasPopup = document.getElementById("recetasPopup");
  const cerrarRecetasButton = document.getElementById("cerrarRecetas");

  // Mostrar el popup de receta electrónica
  if (recetasButton) {
    recetasButton.addEventListener("click", () => {
      recetasPopup.style.display = "flex";
    });
  }

  // Cerrar el popup de receta electrónica
  if (cerrarRecetasButton) {
    cerrarRecetasButton.addEventListener("click", () => {
      recetasPopup.style.display = "none";
    });
  }

  const historialButton = document.querySelector('[data-bs-target="#historialModal"]');
  const comprasPopup = document.getElementById("comprasPopup");
  const cerrarHistorialButton = document.getElementById("cerrarHistorial");

  // Mostrar el popup de historial de compras
  if (historialButton) {
    historialButton.addEventListener("click", () => {
      comprasPopup.style.display = "flex";
    });
  }

  // Cerrar el popup de historial de compras
  if (cerrarHistorialButton) {
    cerrarHistorialButton.addEventListener("click", () => {
      comprasPopup.style.display = "none";
    });
  }
});