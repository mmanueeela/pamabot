/**
 * @jest-environment jsdom
 */

global.TextEncoder = require("util").TextEncoder;
global.TextDecoder = require("util").TextDecoder;


const { JSDOM } = require("jsdom");

describe("script_usuario.js", () => {
  beforeEach(() => {
    global.data = {
      connected: true,
      currentPosition: { x: 0, y: 0, z: 0, w: 1 },
      ros: {},
    };
    global.ROSLIB = {
      Topic: jest.fn().mockImplementation(() => ({
        publish: jest.fn(),
      })),
      Message: jest.fn().mockImplementation((obj) => obj),
    };
    global.console = { log: jest.fn(), warn: jest.fn() };
  });

  test("moveRobot - publica mensaje de avance", () => {
    const { moveRobot, reverse } = require("../js/script_usuario");
    moveRobot("delante");
    expect(console.log).toHaveBeenCalledWith("Moviendo delante:", {
      linear: { x: 0.1, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: 0 },
    });
  });

  test("reverse - cambia la dirección", () => {
    const { moveRobot, reverse } = require("../js/script_usuario");
    reverse();
    expect(console.log).toHaveBeenCalledWith("Dirección cambiada:", "Atrás");
  });
});
