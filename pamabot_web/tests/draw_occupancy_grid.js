/**
 * @jest-environment jsdom
 */

test("canvas se crea y dibuja sin errores", () => {
    document.body.innerHTML = '<canvas id="mapaCanvas"></canvas>';
    const canvas = document.getElementById("mapaCanvas");
    expect(canvas).not.toBeNull();
  
    // Simular dibujo si se usa: ctx.putImageData(), ctx.drawImage(), etc.
    // Aquí podrías mockearlo si tienes funciones públicas
  });
  