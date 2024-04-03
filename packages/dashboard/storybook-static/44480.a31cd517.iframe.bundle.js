/*! For license information please see 44480.a31cd517.iframe.bundle.js.LICENSE.txt */
'use strict';
(self.webpackChunkrmf_dashboard = self.webpackChunkrmf_dashboard || []).push([
  [44480],
  {
    '../../node_modules/.pnpm/troika-three-text@0.49.0_three@0.156.1/node_modules/troika-three-text/dist/troika-three-text.esm.js':
      (__unused_webpack_module, __webpack_exports__, __webpack_require__) => {
        __webpack_require__.r(__webpack_exports__),
          __webpack_require__.d(__webpack_exports__, {
            GlyphsGeometry: () => GlyphsGeometry,
            Text: () => Text,
            configureTextBuilder: () => configureTextBuilder,
            createTextDerivedMaterial: () => createTextDerivedMaterial,
            dumpSDFTextures: () => dumpSDFTextures,
            fontResolverWorkerModule: () => fontResolverWorkerModule,
            getCaretAtPoint: () => getCaretAtPoint,
            getSelectionRects: () => getSelectionRects,
            getTextRenderInfo: () => getTextRenderInfo,
            preloadFont: () => preloadFont,
            typesetterWorkerModule: () => typesetterWorkerModule,
          });
        var three_module = __webpack_require__(
          '../../node_modules/.pnpm/three@0.156.1/node_modules/three/build/three.module.js',
        );
        __webpack_require__(
          '../../node_modules/.pnpm/process@0.11.10/node_modules/process/browser.js',
        );
        function workerBootstrap() {
          var modules = Object.create(null);
          function registerModule(ref, callback) {
            var id = ref.id,
              name = ref.name,
              dependencies = ref.dependencies;
            void 0 === dependencies && (dependencies = []);
            var init = ref.init;
            void 0 === init && (init = function () {});
            var getTransferables = ref.getTransferables;
            if ((void 0 === getTransferables && (getTransferables = null), !modules[id]))
              try {
                (dependencies = dependencies.map(function (dep) {
                  return (
                    dep &&
                      dep.isWorkerModule &&
                      (registerModule(dep, function (depResult) {
                        if (depResult instanceof Error) throw depResult;
                      }),
                      (dep = modules[dep.id].value)),
                    dep
                  );
                })),
                  (init = rehydrate('<' + name + '>.init', init)),
                  getTransferables &&
                    (getTransferables = rehydrate(
                      '<' + name + '>.getTransferables',
                      getTransferables,
                    ));
                var value = null;
                'function' == typeof init
                  ? (value = init.apply(void 0, dependencies))
                  : console.error('worker module init function failed to rehydrate'),
                  (modules[id] = { id, value, getTransferables }),
                  callback(value);
              } catch (err) {
                (err && err.noLog) || console.error(err), callback(err);
              }
          }
          function rehydrate(name, str) {
            var result = void 0;
            self.troikaDefine = function (r) {
              return (result = r);
            };
            var url = URL.createObjectURL(
              new Blob(
                ['/** ' + name.replace(/\*/g, '') + ' **/\n\ntroikaDefine(\n' + str + '\n)'],
                { type: 'application/javascript' },
              ),
            );
            try {
              importScripts(url);
            } catch (err) {
              console.error(err);
            }
            return URL.revokeObjectURL(url), delete self.troikaDefine, result;
          }
          self.addEventListener('message', function (e) {
            var ref = e.data,
              messageId = ref.messageId,
              action = ref.action,
              data = ref.data;
            try {
              'registerModule' === action &&
                registerModule(data, function (result) {
                  result instanceof Error
                    ? postMessage({ messageId, success: !1, error: result.message })
                    : postMessage({
                        messageId,
                        success: !0,
                        result: { isCallable: 'function' == typeof result },
                      });
                }),
                'callModule' === action &&
                  (function callModule(ref, callback) {
                    var ref$1,
                      id = ref.id,
                      args = ref.args;
                    (modules[id] && 'function' == typeof modules[id].value) ||
                      callback(
                        new Error(
                          'Worker module ' +
                            id +
                            ": not found or its 'init' did not return a function",
                        ),
                      );
                    try {
                      var result = (ref$1 = modules[id]).value.apply(ref$1, args);
                      result && 'function' == typeof result.then
                        ? result.then(handleResult, function (rej) {
                            return callback(rej instanceof Error ? rej : new Error('' + rej));
                          })
                        : handleResult(result);
                    } catch (err) {
                      callback(err);
                    }
                    function handleResult(result) {
                      try {
                        var tx =
                          modules[id].getTransferables && modules[id].getTransferables(result);
                        (tx && Array.isArray(tx) && tx.length) || (tx = void 0),
                          callback(result, tx);
                      } catch (err) {
                        console.error(err), callback(err);
                      }
                    }
                  })(data, function (result, transferables) {
                    result instanceof Error
                      ? postMessage({ messageId, success: !1, error: result.message })
                      : postMessage({ messageId, success: !0, result }, transferables || void 0);
                  });
            } catch (err) {
              postMessage({ messageId, success: !1, error: err.stack });
            }
          });
        }
        var supportsWorkers = function () {
            var supported = !1;
            if ('undefined' != typeof window && void 0 !== window.document)
              try {
                new Worker(
                  URL.createObjectURL(new Blob([''], { type: 'application/javascript' })),
                ).terminate(),
                  (supported = !0);
              } catch (err) {
                console.log(
                  'Troika createWorkerModule: web workers not allowed; falling back to main thread execution. Cause: [' +
                    err.message +
                    ']',
                );
              }
            return (
              (supportsWorkers = function () {
                return supported;
              }),
              supported
            );
          },
          _workerModuleId = 0,
          _messageId = 0,
          _allowInitAsString = !1,
          workers = Object.create(null),
          registeredModules = Object.create(null),
          openRequests = Object.create(null);
        function defineWorkerModule(options) {
          if (!((options && 'function' == typeof options.init) || _allowInitAsString))
            throw new Error('requires `options.init` function');
          var dependencies = options.dependencies,
            init = options.init,
            getTransferables = options.getTransferables,
            workerId = options.workerId;
          if (!supportsWorkers())
            return (function defineMainThreadModule(options) {
              var moduleFunc = function () {
                for (var args = [], len = arguments.length; len--; ) args[len] = arguments[len];
                return moduleFunc._getInitResult().then(function (initResult) {
                  if ('function' == typeof initResult) return initResult.apply(void 0, args);
                  throw new Error(
                    'Worker module function was called but `init` did not return a callable function',
                  );
                });
              };
              return (
                (moduleFunc._getInitResult = function () {
                  var dependencies = options.dependencies,
                    init = options.init;
                  dependencies = Array.isArray(dependencies)
                    ? dependencies.map(function (dep) {
                        return dep && dep._getInitResult ? dep._getInitResult() : dep;
                      })
                    : [];
                  var initPromise = Promise.all(dependencies).then(function (deps) {
                    return init.apply(null, deps);
                  });
                  return (
                    (moduleFunc._getInitResult = function () {
                      return initPromise;
                    }),
                    initPromise
                  );
                }),
                moduleFunc
              );
            })(options);
          null == workerId && (workerId = '#default');
          var id = 'workerModule' + ++_workerModuleId,
            name = options.name || id,
            registrationPromise = null;
          function moduleFunc() {
            for (var args = [], len = arguments.length; len--; ) args[len] = arguments[len];
            if (!registrationPromise) {
              registrationPromise = callWorker(
                workerId,
                'registerModule',
                moduleFunc.workerModuleData,
              );
              var unregister = function () {
                (registrationPromise = null), registeredModules[workerId].delete(unregister);
              };
              (registeredModules[workerId] || (registeredModules[workerId] = new Set())).add(
                unregister,
              );
            }
            return registrationPromise.then(function (ref) {
              if (ref.isCallable) return callWorker(workerId, 'callModule', { id, args });
              throw new Error(
                'Worker module function was called but `init` did not return a callable function',
              );
            });
          }
          return (
            (dependencies =
              dependencies &&
              dependencies.map(function (dep) {
                return (
                  'function' != typeof dep ||
                    dep.workerModuleData ||
                    ((_allowInitAsString = !0),
                    (dep = defineWorkerModule({
                      workerId,
                      name: '<' + name + '> function dependency: ' + dep.name,
                      init: 'function(){return (\n' + stringifyFunction(dep) + '\n)}',
                    })),
                    (_allowInitAsString = !1)),
                  dep && dep.workerModuleData && (dep = dep.workerModuleData),
                  dep
                );
              })),
            (moduleFunc.workerModuleData = {
              isWorkerModule: !0,
              id,
              name,
              dependencies,
              init: stringifyFunction(init),
              getTransferables: getTransferables && stringifyFunction(getTransferables),
            }),
            moduleFunc
          );
        }
        function stringifyFunction(fn) {
          var str = fn.toString();
          return !/^function/.test(str) && /^\w+\s*\(/.test(str) && (str = 'function ' + str), str;
        }
        function callWorker(workerId, action, data) {
          return new Promise(function (resolve, reject) {
            var messageId = ++_messageId;
            (openRequests[messageId] = function (response) {
              response.success
                ? resolve(response.result)
                : reject(new Error('Error in worker ' + action + ' call: ' + response.error));
            }),
              (function getWorker(workerId) {
                var worker = workers[workerId];
                if (!worker) {
                  var bootstrap = stringifyFunction(workerBootstrap);
                  (worker = workers[workerId] =
                    new Worker(
                      URL.createObjectURL(
                        new Blob(
                          [
                            '/** Worker Module Bootstrap: ' +
                              workerId.replace(/\*/g, '') +
                              ' **/\n\n;(' +
                              bootstrap +
                              ')()',
                          ],
                          { type: 'application/javascript' },
                        ),
                      ),
                    )).onmessage = function (e) {
                    var response = e.data,
                      msgId = response.messageId,
                      callback = openRequests[msgId];
                    if (!callback)
                      throw new Error('WorkerModule response with empty or unknown messageId');
                    delete openRequests[msgId], callback(response);
                  };
                }
                return worker;
              })(workerId).postMessage({ messageId, action, data });
          });
        }
        function SDFGenerator() {
          var exports = (function (exports) {
            function pointOnCubicBezier(x0, y0, x1, y1, x2, y2, x3, y3, t, pointOut) {
              var t2 = 1 - t;
              (pointOut.x =
                t2 * t2 * t2 * x0 + 3 * t2 * t2 * t * x1 + 3 * t2 * t * t * x2 + t * t * t * x3),
                (pointOut.y =
                  t2 * t2 * t2 * y0 + 3 * t2 * t2 * t * y1 + 3 * t2 * t * t * y2 + t * t * t * y3);
            }
            function forEachPathCommand(pathString, commandCallback) {
              for (
                var match, firstX, firstY, prevX, prevY, segmentRE = /([MLQCZ])([^MLQCZ]*)/g;
                (match = segmentRE.exec(pathString));

              ) {
                var args = match[2]
                  .replace(/^\s*|\s*$/g, '')
                  .split(/[,\s]+/)
                  .map(function (v) {
                    return parseFloat(v);
                  });
                switch (match[1]) {
                  case 'M':
                    (prevX = firstX = args[0]), (prevY = firstY = args[1]);
                    break;
                  case 'L':
                    (args[0] === prevX && args[1] === prevY) ||
                      commandCallback('L', prevX, prevY, (prevX = args[0]), (prevY = args[1]));
                    break;
                  case 'Q':
                    commandCallback(
                      'Q',
                      prevX,
                      prevY,
                      (prevX = args[2]),
                      (prevY = args[3]),
                      args[0],
                      args[1],
                    );
                    break;
                  case 'C':
                    commandCallback(
                      'C',
                      prevX,
                      prevY,
                      (prevX = args[4]),
                      (prevY = args[5]),
                      args[0],
                      args[1],
                      args[2],
                      args[3],
                    );
                    break;
                  case 'Z':
                    (prevX === firstX && prevY === firstY) ||
                      commandCallback('L', prevX, prevY, firstX, firstY);
                }
              }
            }
            function pathToLineSegments(pathString, segmentCallback, curvePoints) {
              void 0 === curvePoints && (curvePoints = 16);
              var tempPoint = { x: 0, y: 0 };
              forEachPathCommand(
                pathString,
                function (command, startX, startY, endX, endY, ctrl1X, ctrl1Y, ctrl2X, ctrl2Y) {
                  switch (command) {
                    case 'L':
                      segmentCallback(startX, startY, endX, endY);
                      break;
                    case 'Q':
                      for (
                        var prevCurveX = startX, prevCurveY = startY, i = 1;
                        i < curvePoints;
                        i++
                      )
                        (y0 = startY),
                          (y1 = ctrl1Y),
                          (y2 = endY),
                          (t2 = void 0),
                          (t2 = 1 - (t = i / (curvePoints - 1))),
                          ((pointOut = tempPoint).x =
                            t2 * t2 * startX + 2 * t2 * t * ctrl1X + t * t * endX),
                          (pointOut.y = t2 * t2 * y0 + 2 * t2 * t * y1 + t * t * y2),
                          segmentCallback(prevCurveX, prevCurveY, tempPoint.x, tempPoint.y),
                          (prevCurveX = tempPoint.x),
                          (prevCurveY = tempPoint.y);
                      break;
                    case 'C':
                      for (
                        var prevCurveX$1 = startX, prevCurveY$1 = startY, i$1 = 1;
                        i$1 < curvePoints;
                        i$1++
                      )
                        pointOnCubicBezier(
                          startX,
                          startY,
                          ctrl1X,
                          ctrl1Y,
                          ctrl2X,
                          ctrl2Y,
                          endX,
                          endY,
                          i$1 / (curvePoints - 1),
                          tempPoint,
                        ),
                          segmentCallback(prevCurveX$1, prevCurveY$1, tempPoint.x, tempPoint.y),
                          (prevCurveX$1 = tempPoint.x),
                          (prevCurveY$1 = tempPoint.y);
                  }
                  var y0, y1, y2, t, pointOut, t2;
                },
              );
            }
            var viewportQuadVertex =
                'precision highp float;attribute vec2 aUV;varying vec2 vUV;void main(){vUV=aUV;gl_Position=vec4(mix(vec2(-1.0),vec2(1.0),aUV),0.0,1.0);}',
              copyTexFragment =
                'precision highp float;uniform sampler2D tex;varying vec2 vUV;void main(){gl_FragColor=texture2D(tex,vUV);}',
              cache = new WeakMap(),
              glContextParams = {
                premultipliedAlpha: !1,
                preserveDrawingBuffer: !0,
                antialias: !1,
                depth: !1,
              };
            function withWebGLContext(glOrCanvas, callback) {
              var gl = glOrCanvas.getContext
                  ? glOrCanvas.getContext('webgl', glContextParams)
                  : glOrCanvas,
                wrapper = cache.get(gl);
              if (!wrapper) {
                var isWebGL2 =
                    'undefined' != typeof WebGL2RenderingContext &&
                    gl instanceof WebGL2RenderingContext,
                  extensions = {},
                  programs = {},
                  textures = {},
                  textureUnit = -1,
                  framebufferStack = [];
                function getExtension(name) {
                  var ext = extensions[name];
                  if (!ext && !(ext = extensions[name] = gl.getExtension(name)))
                    throw new Error(name + ' not supported');
                  return ext;
                }
                function compileShader(src, type) {
                  var shader = gl.createShader(type);
                  return gl.shaderSource(shader, src), gl.compileShader(shader), shader;
                }
                function withProgram(name, vert, frag, func) {
                  if (!programs[name]) {
                    var attributes = {},
                      uniforms = {},
                      program = gl.createProgram();
                    gl.attachShader(program, compileShader(vert, gl.VERTEX_SHADER)),
                      gl.attachShader(program, compileShader(frag, gl.FRAGMENT_SHADER)),
                      gl.linkProgram(program),
                      (programs[name] = {
                        program,
                        transaction: function transaction(func) {
                          gl.useProgram(program),
                            func({
                              setUniform: function setUniform(type, name) {
                                for (var values = [], len = arguments.length - 2; len-- > 0; )
                                  values[len] = arguments[len + 2];
                                var uniformLoc =
                                  uniforms[name] ||
                                  (uniforms[name] = gl.getUniformLocation(program, name));
                                gl['uniform' + type].apply(gl, [uniformLoc].concat(values));
                              },
                              setAttribute: function setAttribute(
                                name,
                                size,
                                usage,
                                instancingDivisor,
                                data,
                              ) {
                                var attr = attributes[name];
                                attr ||
                                  (attr = attributes[name] =
                                    {
                                      buf: gl.createBuffer(),
                                      loc: gl.getAttribLocation(program, name),
                                      data: null,
                                    }),
                                  gl.bindBuffer(gl.ARRAY_BUFFER, attr.buf),
                                  gl.vertexAttribPointer(attr.loc, size, gl.FLOAT, !1, 0, 0),
                                  gl.enableVertexAttribArray(attr.loc),
                                  isWebGL2
                                    ? gl.vertexAttribDivisor(attr.loc, instancingDivisor)
                                    : getExtension(
                                        'ANGLE_instanced_arrays',
                                      ).vertexAttribDivisorANGLE(attr.loc, instancingDivisor),
                                  data !== attr.data &&
                                    (gl.bufferData(gl.ARRAY_BUFFER, data, usage),
                                    (attr.data = data));
                              },
                            });
                        },
                      });
                  }
                  programs[name].transaction(func);
                }
                function withTexture(name, func) {
                  textureUnit++;
                  try {
                    gl.activeTexture(gl.TEXTURE0 + textureUnit);
                    var texture = textures[name];
                    texture ||
                      ((texture = textures[name] = gl.createTexture()),
                      gl.bindTexture(gl.TEXTURE_2D, texture),
                      gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.NEAREST),
                      gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.NEAREST)),
                      gl.bindTexture(gl.TEXTURE_2D, texture),
                      func(texture, textureUnit);
                  } finally {
                    textureUnit--;
                  }
                }
                function withTextureFramebuffer(texture, textureUnit, func) {
                  var framebuffer = gl.createFramebuffer();
                  framebufferStack.push(framebuffer),
                    gl.bindFramebuffer(gl.FRAMEBUFFER, framebuffer),
                    gl.activeTexture(gl.TEXTURE0 + textureUnit),
                    gl.bindTexture(gl.TEXTURE_2D, texture),
                    gl.framebufferTexture2D(
                      gl.FRAMEBUFFER,
                      gl.COLOR_ATTACHMENT0,
                      gl.TEXTURE_2D,
                      texture,
                      0,
                    );
                  try {
                    func(framebuffer);
                  } finally {
                    gl.deleteFramebuffer(framebuffer),
                      gl.bindFramebuffer(
                        gl.FRAMEBUFFER,
                        framebufferStack[--framebufferStack.length - 1] || null,
                      );
                  }
                }
                function handleContextLoss() {
                  (extensions = {}),
                    (programs = {}),
                    (textures = {}),
                    (textureUnit = -1),
                    (framebufferStack.length = 0);
                }
                gl.canvas.addEventListener(
                  'webglcontextlost',
                  function (e) {
                    handleContextLoss(), e.preventDefault();
                  },
                  !1,
                ),
                  cache.set(
                    gl,
                    (wrapper = {
                      gl,
                      isWebGL2,
                      getExtension,
                      withProgram,
                      withTexture,
                      withTextureFramebuffer,
                      handleContextLoss,
                    }),
                  );
              }
              callback(wrapper);
            }
            function renderImageData(
              glOrCanvas,
              imageData,
              x,
              y,
              width,
              height,
              channels,
              framebuffer,
            ) {
              void 0 === channels && (channels = 15),
                void 0 === framebuffer && (framebuffer = null),
                withWebGLContext(glOrCanvas, function (ref) {
                  var gl = ref.gl,
                    withProgram = ref.withProgram;
                  (0, ref.withTexture)('copy', function (tex, texUnit) {
                    gl.texImage2D(
                      gl.TEXTURE_2D,
                      0,
                      gl.RGBA,
                      width,
                      height,
                      0,
                      gl.RGBA,
                      gl.UNSIGNED_BYTE,
                      imageData,
                    ),
                      withProgram('copy', viewportQuadVertex, copyTexFragment, function (ref) {
                        var setUniform = ref.setUniform;
                        (0, ref.setAttribute)(
                          'aUV',
                          2,
                          gl.STATIC_DRAW,
                          0,
                          new Float32Array([0, 0, 2, 0, 0, 2]),
                        ),
                          setUniform('1i', 'image', texUnit),
                          gl.bindFramebuffer(gl.FRAMEBUFFER, framebuffer || null),
                          gl.disable(gl.BLEND),
                          gl.colorMask(8 & channels, 4 & channels, 2 & channels, 1 & channels),
                          gl.viewport(x, y, width, height),
                          gl.scissor(x, y, width, height),
                          gl.drawArrays(gl.TRIANGLES, 0, 3);
                      });
                  });
                });
            }
            var webglUtils = Object.freeze({
              __proto__: null,
              withWebGLContext,
              renderImageData,
              resizeWebGLCanvasWithoutClearing: function resizeWebGLCanvasWithoutClearing(
                canvas,
                newWidth,
                newHeight,
              ) {
                var width = canvas.width,
                  height = canvas.height;
                withWebGLContext(canvas, function (ref) {
                  var gl = ref.gl,
                    data = new Uint8Array(width * height * 4);
                  gl.readPixels(0, 0, width, height, gl.RGBA, gl.UNSIGNED_BYTE, data),
                    (canvas.width = newWidth),
                    (canvas.height = newHeight),
                    renderImageData(gl, data, 0, 0, width, height);
                });
              },
            });
            function generate$2(sdfWidth, sdfHeight, path, viewBox, maxDistance, sdfExponent) {
              void 0 === sdfExponent && (sdfExponent = 1);
              var textureData = new Uint8Array(sdfWidth * sdfHeight),
                viewBoxWidth = viewBox[2] - viewBox[0],
                viewBoxHeight = viewBox[3] - viewBox[1],
                segments = [];
              pathToLineSegments(path, function (x1, y1, x2, y2) {
                segments.push({
                  x1,
                  y1,
                  x2,
                  y2,
                  minX: Math.min(x1, x2),
                  minY: Math.min(y1, y2),
                  maxX: Math.max(x1, x2),
                  maxY: Math.max(y1, y2),
                });
              }),
                segments.sort(function (a, b) {
                  return a.maxX - b.maxX;
                });
              for (var sdfX = 0; sdfX < sdfWidth; sdfX++)
                for (var sdfY = 0; sdfY < sdfHeight; sdfY++) {
                  var signedDist = findNearestSignedDistance(
                      viewBox[0] + (viewBoxWidth * (sdfX + 0.5)) / sdfWidth,
                      viewBox[1] + (viewBoxHeight * (sdfY + 0.5)) / sdfHeight,
                    ),
                    alpha = Math.pow(1 - Math.abs(signedDist) / maxDistance, sdfExponent) / 2;
                  signedDist < 0 && (alpha = 1 - alpha),
                    (alpha = Math.max(0, Math.min(255, Math.round(255 * alpha)))),
                    (textureData[sdfY * sdfWidth + sdfX] = alpha);
                }
              return textureData;
              function findNearestSignedDistance(x, y) {
                for (var closestDistSq = 1 / 0, closestDist = 1 / 0, i = segments.length; i--; ) {
                  var seg = segments[i];
                  if (seg.maxX + closestDist <= x) break;
                  if (
                    x + closestDist > seg.minX &&
                    y - closestDist < seg.maxY &&
                    y + closestDist > seg.minY
                  ) {
                    var distSq = absSquareDistanceToLineSegment(
                      x,
                      y,
                      seg.x1,
                      seg.y1,
                      seg.x2,
                      seg.y2,
                    );
                    distSq < closestDistSq &&
                      ((closestDistSq = distSq), (closestDist = Math.sqrt(closestDistSq)));
                  }
                }
                return (
                  (function isPointInPoly(x, y) {
                    for (var winding = 0, i = segments.length; i--; ) {
                      var seg = segments[i];
                      if (seg.maxX <= x) break;
                      seg.y1 > y != seg.y2 > y &&
                        x < ((seg.x2 - seg.x1) * (y - seg.y1)) / (seg.y2 - seg.y1) + seg.x1 &&
                        (winding += seg.y1 < seg.y2 ? 1 : -1);
                    }
                    return 0 !== winding;
                  })(x, y) && (closestDist = -closestDist),
                  closestDist
                );
              }
            }
            function generateIntoCanvas$2(
              sdfWidth,
              sdfHeight,
              path,
              viewBox,
              maxDistance,
              sdfExponent,
              canvas,
              x,
              y,
              channel,
            ) {
              void 0 === sdfExponent && (sdfExponent = 1),
                void 0 === x && (x = 0),
                void 0 === y && (y = 0),
                void 0 === channel && (channel = 0),
                generateIntoFramebuffer$1(
                  sdfWidth,
                  sdfHeight,
                  path,
                  viewBox,
                  maxDistance,
                  sdfExponent,
                  canvas,
                  null,
                  x,
                  y,
                  channel,
                );
            }
            function generateIntoFramebuffer$1(
              sdfWidth,
              sdfHeight,
              path,
              viewBox,
              maxDistance,
              sdfExponent,
              glOrCanvas,
              framebuffer,
              x,
              y,
              channel,
            ) {
              void 0 === sdfExponent && (sdfExponent = 1),
                void 0 === x && (x = 0),
                void 0 === y && (y = 0),
                void 0 === channel && (channel = 0);
              for (
                var data = generate$2(sdfWidth, sdfHeight, path, viewBox, maxDistance, sdfExponent),
                  rgbaData = new Uint8Array(4 * data.length),
                  i = 0;
                i < data.length;
                i++
              )
                rgbaData[4 * i + channel] = data[i];
              renderImageData(
                glOrCanvas,
                rgbaData,
                x,
                y,
                sdfWidth,
                sdfHeight,
                1 << (3 - channel),
                framebuffer,
              );
            }
            function absSquareDistanceToLineSegment(x, y, lineX0, lineY0, lineX1, lineY1) {
              var ldx = lineX1 - lineX0,
                ldy = lineY1 - lineY0,
                lengthSq = ldx * ldx + ldy * ldy,
                t = lengthSq
                  ? Math.max(0, Math.min(1, ((x - lineX0) * ldx + (y - lineY0) * ldy) / lengthSq))
                  : 0,
                dx = x - (lineX0 + t * ldx),
                dy = y - (lineY0 + t * ldy);
              return dx * dx + dy * dy;
            }
            var javascript = Object.freeze({
                __proto__: null,
                generate: generate$2,
                generateIntoCanvas: generateIntoCanvas$2,
                generateIntoFramebuffer: generateIntoFramebuffer$1,
              }),
              mainVertex =
                'precision highp float;uniform vec4 uGlyphBounds;attribute vec2 aUV;attribute vec4 aLineSegment;varying vec4 vLineSegment;varying vec2 vGlyphXY;void main(){vLineSegment=aLineSegment;vGlyphXY=mix(uGlyphBounds.xy,uGlyphBounds.zw,aUV);gl_Position=vec4(mix(vec2(-1.0),vec2(1.0),aUV),0.0,1.0);}',
              mainFragment =
                'precision highp float;uniform vec4 uGlyphBounds;uniform float uMaxDistance;uniform float uExponent;varying vec4 vLineSegment;varying vec2 vGlyphXY;float absDistToSegment(vec2 point,vec2 lineA,vec2 lineB){vec2 lineDir=lineB-lineA;float lenSq=dot(lineDir,lineDir);float t=lenSq==0.0 ? 0.0 : clamp(dot(point-lineA,lineDir)/lenSq,0.0,1.0);vec2 linePt=lineA+t*lineDir;return distance(point,linePt);}void main(){vec4 seg=vLineSegment;vec2 p=vGlyphXY;float dist=absDistToSegment(p,seg.xy,seg.zw);float val=pow(1.0-clamp(dist/uMaxDistance,0.0,1.0),uExponent)*0.5;bool crossing=(seg.y>p.y!=seg.w>p.y)&&(p.x<(seg.z-seg.x)*(p.y-seg.y)/(seg.w-seg.y)+seg.x);bool crossingUp=crossing&&vLineSegment.y<vLineSegment.w;gl_FragColor=vec4(crossingUp ? 1.0/255.0 : 0.0,crossing&&!crossingUp ? 1.0/255.0 : 0.0,0.0,val);}',
              postFragment =
                'precision highp float;uniform sampler2D tex;varying vec2 vUV;void main(){vec4 color=texture2D(tex,vUV);bool inside=color.r!=color.g;float val=inside ? 1.0-color.a : color.a;gl_FragColor=vec4(val);}',
              viewportUVs = new Float32Array([0, 0, 2, 0, 0, 2]),
              implicitContext = null,
              isTestingSupport = !1,
              NULL_OBJECT = {},
              supportByCanvas = new WeakMap();
            function validateSupport(glOrCanvas) {
              if (!isTestingSupport && !isSupported(glOrCanvas))
                throw new Error('WebGL generation not supported');
            }
            function generate$1(
              sdfWidth,
              sdfHeight,
              path,
              viewBox,
              maxDistance,
              sdfExponent,
              glOrCanvas,
            ) {
              if (
                (void 0 === sdfExponent && (sdfExponent = 1),
                void 0 === glOrCanvas && (glOrCanvas = null),
                !glOrCanvas && !(glOrCanvas = implicitContext))
              ) {
                var canvas =
                  'function' == typeof OffscreenCanvas
                    ? new OffscreenCanvas(1, 1)
                    : 'undefined' != typeof document
                      ? document.createElement('canvas')
                      : null;
                if (!canvas) throw new Error('OffscreenCanvas or DOM canvas not supported');
                glOrCanvas = implicitContext = canvas.getContext('webgl', { depth: !1 });
              }
              validateSupport(glOrCanvas);
              var rgbaData = new Uint8Array(sdfWidth * sdfHeight * 4);
              withWebGLContext(glOrCanvas, function (ref) {
                var gl = ref.gl,
                  withTexture = ref.withTexture,
                  withTextureFramebuffer = ref.withTextureFramebuffer;
                withTexture('readable', function (texture, textureUnit) {
                  gl.texImage2D(
                    gl.TEXTURE_2D,
                    0,
                    gl.RGBA,
                    sdfWidth,
                    sdfHeight,
                    0,
                    gl.RGBA,
                    gl.UNSIGNED_BYTE,
                    null,
                  ),
                    withTextureFramebuffer(texture, textureUnit, function (framebuffer) {
                      generateIntoFramebuffer(
                        sdfWidth,
                        sdfHeight,
                        path,
                        viewBox,
                        maxDistance,
                        sdfExponent,
                        gl,
                        framebuffer,
                        0,
                        0,
                        0,
                      ),
                        gl.readPixels(
                          0,
                          0,
                          sdfWidth,
                          sdfHeight,
                          gl.RGBA,
                          gl.UNSIGNED_BYTE,
                          rgbaData,
                        );
                    });
                });
              });
              for (
                var data = new Uint8Array(sdfWidth * sdfHeight), i = 0, j = 0;
                i < rgbaData.length;
                i += 4
              )
                data[j++] = rgbaData[i];
              return data;
            }
            function generateIntoCanvas$1(
              sdfWidth,
              sdfHeight,
              path,
              viewBox,
              maxDistance,
              sdfExponent,
              canvas,
              x,
              y,
              channel,
            ) {
              void 0 === sdfExponent && (sdfExponent = 1),
                void 0 === x && (x = 0),
                void 0 === y && (y = 0),
                void 0 === channel && (channel = 0),
                generateIntoFramebuffer(
                  sdfWidth,
                  sdfHeight,
                  path,
                  viewBox,
                  maxDistance,
                  sdfExponent,
                  canvas,
                  null,
                  x,
                  y,
                  channel,
                );
            }
            function generateIntoFramebuffer(
              sdfWidth,
              sdfHeight,
              path,
              viewBox,
              maxDistance,
              sdfExponent,
              glOrCanvas,
              framebuffer,
              x,
              y,
              channel,
            ) {
              void 0 === sdfExponent && (sdfExponent = 1),
                void 0 === x && (x = 0),
                void 0 === y && (y = 0),
                void 0 === channel && (channel = 0),
                validateSupport(glOrCanvas);
              var lineSegmentCoords = [];
              pathToLineSegments(path, function (x1, y1, x2, y2) {
                lineSegmentCoords.push(x1, y1, x2, y2);
              }),
                (lineSegmentCoords = new Float32Array(lineSegmentCoords)),
                withWebGLContext(glOrCanvas, function (ref) {
                  var gl = ref.gl,
                    isWebGL2 = ref.isWebGL2,
                    getExtension = ref.getExtension,
                    withProgram = ref.withProgram,
                    withTexture = ref.withTexture,
                    withTextureFramebuffer = ref.withTextureFramebuffer,
                    handleContextLoss = ref.handleContextLoss;
                  if (
                    (withTexture(
                      'rawDistances',
                      function (intermediateTexture, intermediateTextureUnit) {
                        (sdfWidth === intermediateTexture._lastWidth &&
                          sdfHeight === intermediateTexture._lastHeight) ||
                          gl.texImage2D(
                            gl.TEXTURE_2D,
                            0,
                            gl.RGBA,
                            (intermediateTexture._lastWidth = sdfWidth),
                            (intermediateTexture._lastHeight = sdfHeight),
                            0,
                            gl.RGBA,
                            gl.UNSIGNED_BYTE,
                            null,
                          ),
                          withProgram('main', mainVertex, mainFragment, function (ref) {
                            var setAttribute = ref.setAttribute,
                              setUniform = ref.setUniform,
                              instancingExtension =
                                !isWebGL2 && getExtension('ANGLE_instanced_arrays'),
                              blendMinMaxExtension = !isWebGL2 && getExtension('EXT_blend_minmax');
                            setAttribute('aUV', 2, gl.STATIC_DRAW, 0, viewportUVs),
                              setAttribute(
                                'aLineSegment',
                                4,
                                gl.DYNAMIC_DRAW,
                                1,
                                lineSegmentCoords,
                              ),
                              setUniform.apply(void 0, ['4f', 'uGlyphBounds'].concat(viewBox)),
                              setUniform('1f', 'uMaxDistance', maxDistance),
                              setUniform('1f', 'uExponent', sdfExponent),
                              withTextureFramebuffer(
                                intermediateTexture,
                                intermediateTextureUnit,
                                function (framebuffer) {
                                  gl.enable(gl.BLEND),
                                    gl.colorMask(!0, !0, !0, !0),
                                    gl.viewport(0, 0, sdfWidth, sdfHeight),
                                    gl.scissor(0, 0, sdfWidth, sdfHeight),
                                    gl.blendFunc(gl.ONE, gl.ONE),
                                    gl.blendEquationSeparate(
                                      gl.FUNC_ADD,
                                      isWebGL2 ? gl.MAX : blendMinMaxExtension.MAX_EXT,
                                    ),
                                    gl.clear(gl.COLOR_BUFFER_BIT),
                                    isWebGL2
                                      ? gl.drawArraysInstanced(
                                          gl.TRIANGLES,
                                          0,
                                          3,
                                          lineSegmentCoords.length / 4,
                                        )
                                      : instancingExtension.drawArraysInstancedANGLE(
                                          gl.TRIANGLES,
                                          0,
                                          3,
                                          lineSegmentCoords.length / 4,
                                        );
                                },
                              );
                          }),
                          withProgram('post', viewportQuadVertex, postFragment, function (program) {
                            program.setAttribute('aUV', 2, gl.STATIC_DRAW, 0, viewportUVs),
                              program.setUniform('1i', 'tex', intermediateTextureUnit),
                              gl.bindFramebuffer(gl.FRAMEBUFFER, framebuffer),
                              gl.disable(gl.BLEND),
                              gl.colorMask(
                                0 === channel,
                                1 === channel,
                                2 === channel,
                                3 === channel,
                              ),
                              gl.viewport(x, y, sdfWidth, sdfHeight),
                              gl.scissor(x, y, sdfWidth, sdfHeight),
                              gl.drawArrays(gl.TRIANGLES, 0, 3);
                          });
                      },
                    ),
                    gl.isContextLost())
                  )
                    throw (handleContextLoss(), new Error('webgl context lost'));
                });
            }
            function isSupported(glOrCanvas) {
              var key =
                  glOrCanvas && glOrCanvas !== implicitContext
                    ? glOrCanvas.canvas || glOrCanvas
                    : NULL_OBJECT,
                supported = supportByCanvas.get(key);
              if (void 0 === supported) {
                isTestingSupport = !0;
                var failReason = null;
                try {
                  var expectedResult = [
                      97, 106, 97, 61, 99, 137, 118, 80, 80, 118, 137, 99, 61, 97, 106, 97,
                    ],
                    testResult = generate$1(
                      4,
                      4,
                      'M8,8L16,8L24,24L16,24Z',
                      [0, 0, 32, 32],
                      24,
                      1,
                      glOrCanvas,
                    );
                  (supported =
                    testResult &&
                    expectedResult.length === testResult.length &&
                    testResult.every(function (val, i) {
                      return val === expectedResult[i];
                    })) ||
                    ((failReason = 'bad trial run results'),
                    console.info(expectedResult, testResult));
                } catch (err) {
                  (supported = !1), (failReason = err.message);
                }
                failReason && console.warn('WebGL SDF generation not supported:', failReason),
                  (isTestingSupport = !1),
                  supportByCanvas.set(key, supported);
              }
              return supported;
            }
            var webgl = Object.freeze({
              __proto__: null,
              generate: generate$1,
              generateIntoCanvas: generateIntoCanvas$1,
              generateIntoFramebuffer,
              isSupported,
            });
            return (
              (exports.forEachPathCommand = forEachPathCommand),
              (exports.generate = function generate(
                sdfWidth,
                sdfHeight,
                path,
                viewBox,
                maxDistance,
                sdfExponent,
              ) {
                void 0 === maxDistance &&
                  (maxDistance = Math.max(viewBox[2] - viewBox[0], viewBox[3] - viewBox[1]) / 2),
                  void 0 === sdfExponent && (sdfExponent = 1);
                try {
                  return generate$1.apply(webgl, arguments);
                } catch (e) {
                  return (
                    console.info('WebGL SDF generation failed, falling back to JS', e),
                    generate$2.apply(javascript, arguments)
                  );
                }
              }),
              (exports.generateIntoCanvas = function generateIntoCanvas(
                sdfWidth,
                sdfHeight,
                path,
                viewBox,
                maxDistance,
                sdfExponent,
                canvas,
                x,
                y,
                channel,
              ) {
                void 0 === maxDistance &&
                  (maxDistance = Math.max(viewBox[2] - viewBox[0], viewBox[3] - viewBox[1]) / 2),
                  void 0 === sdfExponent && (sdfExponent = 1),
                  void 0 === x && (x = 0),
                  void 0 === y && (y = 0),
                  void 0 === channel && (channel = 0);
                try {
                  return generateIntoCanvas$1.apply(webgl, arguments);
                } catch (e) {
                  return (
                    console.info('WebGL SDF generation failed, falling back to JS', e),
                    generateIntoCanvas$2.apply(javascript, arguments)
                  );
                }
              }),
              (exports.javascript = javascript),
              (exports.pathToLineSegments = pathToLineSegments),
              (exports.webgl = webgl),
              (exports.webglUtils = webglUtils),
              Object.defineProperty(exports, '__esModule', { value: !0 }),
              exports
            );
          })({});
          return exports;
        }
        const bidi = function bidiFactory() {
            return (function (exports) {
              var DATA = {
                  R: '13k,1a,2,3,3,2+1j,ch+16,a+1,5+2,2+n,5,a,4,6+16,4+3,h+1b,4mo,179q,2+9,2+11,2i9+7y,2+68,4,3+4,5+13,4+3,2+4k,3+29,8+cf,1t+7z,w+17,3+3m,1t+3z,16o1+5r,8+30,8+mc,29+1r,29+4v,75+73',
                  EN: '1c+9,3d+1,6,187+9,513,4+5,7+9,sf+j,175h+9,qw+q,161f+1d,4xt+a,25i+9',
                  ES: '17,2,6dp+1,f+1,av,16vr,mx+1,4o,2',
                  ET: 'z+2,3h+3,b+1,ym,3e+1,2o,p4+1,8,6u,7c,g6,1wc,1n9+4,30+1b,2n,6d,qhx+1,h0m,a+1,49+2,63+1,4+1,6bb+3,12jj',
                  AN: '16o+5,2j+9,2+1,35,ed,1ff2+9,87+u',
                  CS: '18,2+1,b,2u,12k,55v,l,17v0,2,3,53,2+1,b',
                  B: 'a,3,f+2,2v,690',
                  S: '9,2,k',
                  WS: 'c,k,4f4,1vk+a,u,1j,335',
                  ON: 'x+1,4+4,h+5,r+5,r+3,z,5+3,2+1,2+1,5,2+2,3+4,o,w,ci+1,8+d,3+d,6+8,2+g,39+1,9,6+1,2,33,b8,3+1,3c+1,7+1,5r,b,7h+3,sa+5,2,3i+6,jg+3,ur+9,2v,ij+1,9g+9,7+a,8m,4+1,49+x,14u,2+2,c+2,e+2,e+2,e+1,i+n,e+e,2+p,u+2,e+2,36+1,2+3,2+1,b,2+2,6+5,2,2,2,h+1,5+4,6+3,3+f,16+2,5+3l,3+81,1y+p,2+40,q+a,m+13,2r+ch,2+9e,75+hf,3+v,2+2w,6e+5,f+6,75+2a,1a+p,2+2g,d+5x,r+b,6+3,4+o,g,6+1,6+2,2k+1,4,2j,5h+z,1m+1,1e+f,t+2,1f+e,d+3,4o+3,2s+1,w,535+1r,h3l+1i,93+2,2s,b+1,3l+x,2v,4g+3,21+3,kz+1,g5v+1,5a,j+9,n+v,2,3,2+8,2+1,3+2,2,3,46+1,4+4,h+5,r+5,r+a,3h+2,4+6,b+4,78,1r+24,4+c,4,1hb,ey+6,103+j,16j+c,1ux+7,5+g,fsh,jdq+1t,4,57+2e,p1,1m,1m,1m,1m,4kt+1,7j+17,5+2r,d+e,3+e,2+e,2+10,m+4,w,1n+5,1q,4z+5,4b+rb,9+c,4+c,4+37,d+2g,8+b,l+b,5+1j,9+9,7+13,9+t,3+1,27+3c,2+29,2+3q,d+d,3+4,4+2,6+6,a+o,8+6,a+2,e+6,16+42,2+1i',
                  BN: '0+8,6+d,2s+5,2+p,e,4m9,1kt+2,2b+5,5+5,17q9+v,7k,6p+8,6+1,119d+3,440+7,96s+1,1ekf+1,1ekf+1,1ekf+1,1ekf+1,1ekf+1,1ekf+1,1ekf+1,1ekf+1,1ekf+1,1ekf+1,1ekf+1,1ekf+75,6p+2rz,1ben+1,1ekf+1,1ekf+1',
                  NSM: 'lc+33,7o+6,7c+18,2,2+1,2+1,2,21+a,1d+k,h,2u+6,3+5,3+1,2+3,10,v+q,2k+a,1n+8,a,p+3,2+8,2+2,2+4,18+2,3c+e,2+v,1k,2,5+7,5,4+6,b+1,u,1n,5+3,9,l+1,r,3+1,1m,5+1,5+1,3+2,4,v+1,4,c+1,1m,5+4,2+1,5,l+1,n+5,2,1n,3,2+3,9,8+1,c+1,v,1q,d,1f,4,1m+2,6+2,2+3,8+1,c+1,u,1n,g+1,l+1,t+1,1m+1,5+3,9,l+1,u,21,8+2,2,2j,3+6,d+7,2r,3+8,c+5,23+1,s,2,2,1k+d,2+4,2+1,6+a,2+z,a,2v+3,2+5,2+1,3+1,q+1,5+2,h+3,e,3+1,7,g,jk+2,qb+2,u+2,u+1,v+1,1t+1,2+6,9,3+a,a,1a+2,3c+1,z,3b+2,5+1,a,7+2,64+1,3,1n,2+6,2,2,3+7,7+9,3,1d+g,1s+3,1d,2+4,2,6,15+8,d+1,x+3,3+1,2+2,1l,2+1,4,2+2,1n+7,3+1,49+2,2+c,2+6,5,7,4+1,5j+1l,2+4,k1+w,2db+2,3y,2p+v,ff+3,30+1,n9x+3,2+9,x+1,29+1,7l,4,5,q+1,6,48+1,r+h,e,13+7,q+a,1b+2,1d,3+3,3+1,14,1w+5,3+1,3+1,d,9,1c,1g,2+2,3+1,6+1,2,17+1,9,6n,3,5,fn5,ki+f,h+f,r2,6b,46+4,1af+2,2+1,6+3,15+2,5,4m+1,fy+3,as+1,4a+a,4x,1j+e,1l+2,1e+3,3+1,1y+2,11+4,2+7,1r,d+1,1h+8,b+3,3,2o+2,3,2+1,7,4h,4+7,m+1,1m+1,4,12+6,4+4,5g+7,3+2,2,o,2d+5,2,5+1,2+1,6n+3,7+1,2+1,s+1,2e+7,3,2+1,2z,2,3+5,2,2u+2,3+3,2+4,78+8,2+1,75+1,2,5,41+3,3+1,5,x+5,3+1,15+5,3+3,9,a+5,3+2,1b+c,2+1,bb+6,2+5,2d+l,3+6,2+1,2+1,3f+5,4,2+1,2+6,2,21+1,4,2,9o+1,f0c+4,1o+6,t5,1s+3,2a,f5l+1,43t+2,i+7,3+6,v+3,45+2,1j0+1i,5+1d,9,f,n+4,2+e,11t+6,2+g,3+6,2+1,2+4,7a+6,c6+3,15t+6,32+6,gzhy+6n',
                  AL: '16w,3,2,e+1b,z+2,2+2s,g+1,8+1,b+m,2+t,s+2i,c+e,4h+f,1d+1e,1bwe+dp,3+3z,x+c,2+1,35+3y,2rm+z,5+7,b+5,dt+l,c+u,17nl+27,1t+27,4x+6n,3+d',
                  LRO: '6ct',
                  RLO: '6cu',
                  LRE: '6cq',
                  RLE: '6cr',
                  PDF: '6cs',
                  LRI: '6ee',
                  RLI: '6ef',
                  FSI: '6eg',
                  PDI: '6eh',
                },
                TYPES = {},
                TYPES_TO_NAMES = {};
              (TYPES.L = 1),
                (TYPES_TO_NAMES[1] = 'L'),
                Object.keys(DATA).forEach(function (type, i) {
                  (TYPES[type] = 1 << (i + 1)), (TYPES_TO_NAMES[TYPES[type]] = type);
                }),
                Object.freeze(TYPES);
              var ISOLATE_INIT_TYPES = TYPES.LRI | TYPES.RLI | TYPES.FSI,
                STRONG_TYPES = TYPES.L | TYPES.R | TYPES.AL,
                NEUTRAL_ISOLATE_TYPES =
                  TYPES.B |
                  TYPES.S |
                  TYPES.WS |
                  TYPES.ON |
                  TYPES.FSI |
                  TYPES.LRI |
                  TYPES.RLI |
                  TYPES.PDI,
                BN_LIKE_TYPES =
                  TYPES.BN | TYPES.RLE | TYPES.LRE | TYPES.RLO | TYPES.LRO | TYPES.PDF,
                TRAILING_TYPES =
                  TYPES.S | TYPES.WS | TYPES.B | ISOLATE_INIT_TYPES | TYPES.PDI | BN_LIKE_TYPES,
                map = null;
              function getBidiCharType(char) {
                return (
                  (function parseData() {
                    if (!map) {
                      map = new Map();
                      var loop = function (type) {
                        if (DATA.hasOwnProperty(type)) {
                          var lastCode = 0;
                          DATA[type].split(',').forEach(function (range) {
                            var ref = range.split('+'),
                              skip = ref[0],
                              step = ref[1];
                            (skip = parseInt(skip, 36)),
                              (step = step ? parseInt(step, 36) : 0),
                              map.set((lastCode += skip), TYPES[type]);
                            for (var i = 0; i < step; i++) map.set(++lastCode, TYPES[type]);
                          });
                        }
                      };
                      for (var type in DATA) loop(type);
                    }
                  })(),
                  map.get(char.codePointAt(0)) || TYPES.L
                );
              }
              var openToClose,
                closeToOpen,
                canonical,
                data$1 = {
                  pairs:
                    '14>1,1e>2,u>2,2wt>1,1>1,1ge>1,1wp>1,1j>1,f>1,hm>1,1>1,u>1,u6>1,1>1,+5,28>1,w>1,1>1,+3,b8>1,1>1,+3,1>3,-1>-1,3>1,1>1,+2,1s>1,1>1,x>1,th>1,1>1,+2,db>1,1>1,+3,3>1,1>1,+2,14qm>1,1>1,+1,4q>1,1e>2,u>2,2>1,+1',
                  canonical:
                    '6f1>-6dx,6dy>-6dx,6ec>-6ed,6ee>-6ed,6ww>2jj,-2ji>2jj,14r4>-1e7l,1e7m>-1e7l,1e7m>-1e5c,1e5d>-1e5b,1e5c>-14qx,14qy>-14qx,14vn>-1ecg,1ech>-1ecg,1edu>-1ecg,1eci>-1ecg,1eda>-1ecg,1eci>-1ecg,1eci>-168q,168r>-168q,168s>-14ye,14yf>-14ye',
                };
              function parseCharacterMap(encodedString, includeReverse) {
                var prevPair,
                  lastCode = 0,
                  map = new Map(),
                  reverseMap = includeReverse && new Map();
                return (
                  encodedString.split(',').forEach(function visit(entry) {
                    if (-1 !== entry.indexOf('+')) for (var i = +entry; i--; ) visit(prevPair);
                    else {
                      prevPair = entry;
                      var ref = entry.split('>'),
                        a = ref[0],
                        b = ref[1];
                      (a = String.fromCodePoint((lastCode += parseInt(a, 36)))),
                        (b = String.fromCodePoint((lastCode += parseInt(b, 36)))),
                        map.set(a, b),
                        includeReverse && reverseMap.set(b, a);
                    }
                  }),
                  { map, reverseMap }
                );
              }
              function parse$1() {
                if (!openToClose) {
                  var ref = parseCharacterMap(data$1.pairs, !0),
                    map = ref.map,
                    reverseMap = ref.reverseMap;
                  (openToClose = map),
                    (closeToOpen = reverseMap),
                    (canonical = parseCharacterMap(data$1.canonical, !1).map);
                }
              }
              function openingToClosingBracket(char) {
                return parse$1(), openToClose.get(char) || null;
              }
              function closingToOpeningBracket(char) {
                return parse$1(), closeToOpen.get(char) || null;
              }
              function getCanonicalBracket(char) {
                return parse$1(), canonical.get(char) || null;
              }
              var TYPE_L = TYPES.L,
                TYPE_R = TYPES.R,
                TYPE_EN = TYPES.EN,
                TYPE_ES = TYPES.ES,
                TYPE_ET = TYPES.ET,
                TYPE_AN = TYPES.AN,
                TYPE_CS = TYPES.CS,
                TYPE_B = TYPES.B,
                TYPE_S = TYPES.S,
                TYPE_ON = TYPES.ON,
                TYPE_BN = TYPES.BN,
                TYPE_NSM = TYPES.NSM,
                TYPE_AL = TYPES.AL,
                TYPE_LRO = TYPES.LRO,
                TYPE_RLO = TYPES.RLO,
                TYPE_LRE = TYPES.LRE,
                TYPE_RLE = TYPES.RLE,
                TYPE_PDF = TYPES.PDF,
                TYPE_LRI = TYPES.LRI,
                TYPE_RLI = TYPES.RLI,
                TYPE_FSI = TYPES.FSI,
                TYPE_PDI = TYPES.PDI;
              var mirrorMap,
                data =
                  '14>1,j>2,t>2,u>2,1a>g,2v3>1,1>1,1ge>1,1wd>1,b>1,1j>1,f>1,ai>3,-2>3,+1,8>1k0,-1jq>1y7,-1y6>1hf,-1he>1h6,-1h5>1ha,-1h8>1qi,-1pu>1,6>3u,-3s>7,6>1,1>1,f>1,1>1,+2,3>1,1>1,+13,4>1,1>1,6>1eo,-1ee>1,3>1mg,-1me>1mk,-1mj>1mi,-1mg>1mi,-1md>1,1>1,+2,1>10k,-103>1,1>1,4>1,5>1,1>1,+10,3>1,1>8,-7>8,+1,-6>7,+1,a>1,1>1,u>1,u6>1,1>1,+5,26>1,1>1,2>1,2>2,8>1,7>1,4>1,1>1,+5,b8>1,1>1,+3,1>3,-2>1,2>1,1>1,+2,c>1,3>1,1>1,+2,h>1,3>1,a>1,1>1,2>1,3>1,1>1,d>1,f>1,3>1,1a>1,1>1,6>1,7>1,13>1,k>1,1>1,+19,4>1,1>1,+2,2>1,1>1,+18,m>1,a>1,1>1,lk>1,1>1,4>1,2>1,f>1,3>1,1>1,+3,db>1,1>1,+3,3>1,1>1,+2,14qm>1,1>1,+1,6>1,4j>1,j>2,t>2,u>2,2>1,+1';
              function getMirroredCharacter(char) {
                return (
                  (function parse() {
                    if (!mirrorMap) {
                      var ref = parseCharacterMap(data, !0),
                        map = ref.map;
                      ref.reverseMap.forEach(function (value, key) {
                        map.set(key, value);
                      }),
                        (mirrorMap = map);
                    }
                  })(),
                  mirrorMap.get(char) || null
                );
              }
              function getReorderSegments(string, embeddingLevelsResult, start, end) {
                var strLen = string.length;
                (start = Math.max(0, null == start ? 0 : +start)),
                  (end = Math.min(strLen - 1, null == end ? strLen - 1 : +end));
                var segments = [];
                return (
                  embeddingLevelsResult.paragraphs.forEach(function (paragraph) {
                    var lineStart = Math.max(start, paragraph.start),
                      lineEnd = Math.min(end, paragraph.end);
                    if (lineStart < lineEnd) {
                      for (
                        var lineLevels = embeddingLevelsResult.levels.slice(lineStart, lineEnd + 1),
                          i = lineEnd;
                        i >= lineStart && getBidiCharType(string[i]) & TRAILING_TYPES;
                        i--
                      )
                        lineLevels[i] = paragraph.level;
                      for (
                        var maxLevel = paragraph.level, minOddLevel = 1 / 0, i$1 = 0;
                        i$1 < lineLevels.length;
                        i$1++
                      ) {
                        var level = lineLevels[i$1];
                        level > maxLevel && (maxLevel = level),
                          level < minOddLevel && (minOddLevel = 1 | level);
                      }
                      for (var lvl = maxLevel; lvl >= minOddLevel; lvl--)
                        for (var i$2 = 0; i$2 < lineLevels.length; i$2++)
                          if (lineLevels[i$2] >= lvl) {
                            for (
                              var segStart = i$2;
                              i$2 + 1 < lineLevels.length && lineLevels[i$2 + 1] >= lvl;

                            )
                              i$2++;
                            i$2 > segStart &&
                              segments.push([segStart + lineStart, i$2 + lineStart]);
                          }
                    }
                  }),
                  segments
                );
              }
              function getReorderedIndices(string, embedLevelsResult, start, end) {
                for (
                  var segments = getReorderSegments(string, embedLevelsResult, start, end),
                    indices = [],
                    i = 0;
                  i < string.length;
                  i++
                )
                  indices[i] = i;
                return (
                  segments.forEach(function (ref) {
                    for (
                      var start = ref[0],
                        end = ref[1],
                        slice = indices.slice(start, end + 1),
                        i = slice.length;
                      i--;

                    )
                      indices[end - i] = slice[i];
                  }),
                  indices
                );
              }
              return (
                (exports.closingToOpeningBracket = closingToOpeningBracket),
                (exports.getBidiCharType = getBidiCharType),
                (exports.getBidiCharTypeName = function getBidiCharTypeName(char) {
                  return TYPES_TO_NAMES[getBidiCharType(char)];
                }),
                (exports.getCanonicalBracket = getCanonicalBracket),
                (exports.getEmbeddingLevels = function getEmbeddingLevels(string, baseDirection) {
                  for (
                    var charTypes = new Uint32Array(string.length), i = 0;
                    i < string.length;
                    i++
                  )
                    charTypes[i] = getBidiCharType(string[i]);
                  var charTypeCounts = new Map();
                  function changeCharType(i, type) {
                    var oldType = charTypes[i];
                    (charTypes[i] = type),
                      charTypeCounts.set(oldType, charTypeCounts.get(oldType) - 1),
                      oldType & NEUTRAL_ISOLATE_TYPES &&
                        charTypeCounts.set(
                          NEUTRAL_ISOLATE_TYPES,
                          charTypeCounts.get(NEUTRAL_ISOLATE_TYPES) - 1,
                        ),
                      charTypeCounts.set(type, (charTypeCounts.get(type) || 0) + 1),
                      type & NEUTRAL_ISOLATE_TYPES &&
                        charTypeCounts.set(
                          NEUTRAL_ISOLATE_TYPES,
                          (charTypeCounts.get(NEUTRAL_ISOLATE_TYPES) || 0) + 1,
                        );
                  }
                  for (
                    var embedLevels = new Uint8Array(string.length),
                      isolationPairs = new Map(),
                      paragraphs = [],
                      paragraph = null,
                      i$1 = 0;
                    i$1 < string.length;
                    i$1++
                  )
                    paragraph ||
                      paragraphs.push(
                        (paragraph = {
                          start: i$1,
                          end: string.length - 1,
                          level:
                            'rtl' === baseDirection
                              ? 1
                              : 'ltr' === baseDirection
                                ? 0
                                : determineAutoEmbedLevel(i$1, !1),
                        }),
                      ),
                      charTypes[i$1] & TYPE_B && ((paragraph.end = i$1), (paragraph = null));
                  for (
                    var FORMATTING_TYPES =
                        TYPE_RLE |
                        TYPE_LRE |
                        TYPE_RLO |
                        TYPE_LRO |
                        ISOLATE_INIT_TYPES |
                        TYPE_PDI |
                        TYPE_PDF |
                        TYPE_B,
                      nextEven = function (n) {
                        return n + (1 & n ? 1 : 2);
                      },
                      nextOdd = function (n) {
                        return n + (1 & n ? 2 : 1);
                      },
                      paraIdx = 0;
                    paraIdx < paragraphs.length;
                    paraIdx++
                  ) {
                    var statusStack = [
                        {
                          _level: (paragraph = paragraphs[paraIdx]).level,
                          _override: 0,
                          _isolate: 0,
                        },
                      ],
                      stackTop = void 0,
                      overflowIsolateCount = 0,
                      overflowEmbeddingCount = 0,
                      validIsolateCount = 0;
                    charTypeCounts.clear();
                    for (var i$2 = paragraph.start; i$2 <= paragraph.end; i$2++) {
                      var charType = charTypes[i$2];
                      if (
                        ((stackTop = statusStack[statusStack.length - 1]),
                        charTypeCounts.set(charType, (charTypeCounts.get(charType) || 0) + 1),
                        charType & NEUTRAL_ISOLATE_TYPES &&
                          charTypeCounts.set(
                            NEUTRAL_ISOLATE_TYPES,
                            (charTypeCounts.get(NEUTRAL_ISOLATE_TYPES) || 0) + 1,
                          ),
                        charType & FORMATTING_TYPES)
                      )
                        if (charType & (TYPE_RLE | TYPE_LRE)) {
                          embedLevels[i$2] = stackTop._level;
                          var level = (charType === TYPE_RLE ? nextOdd : nextEven)(stackTop._level);
                          level <= 125 && !overflowIsolateCount && !overflowEmbeddingCount
                            ? statusStack.push({ _level: level, _override: 0, _isolate: 0 })
                            : overflowIsolateCount || overflowEmbeddingCount++;
                        } else if (charType & (TYPE_RLO | TYPE_LRO)) {
                          embedLevels[i$2] = stackTop._level;
                          var level$1 = (charType === TYPE_RLO ? nextOdd : nextEven)(
                            stackTop._level,
                          );
                          level$1 <= 125 && !overflowIsolateCount && !overflowEmbeddingCount
                            ? statusStack.push({
                                _level: level$1,
                                _override: charType & TYPE_RLO ? TYPE_R : TYPE_L,
                                _isolate: 0,
                              })
                            : overflowIsolateCount || overflowEmbeddingCount++;
                        } else if (charType & ISOLATE_INIT_TYPES) {
                          charType & TYPE_FSI &&
                            (charType =
                              1 === determineAutoEmbedLevel(i$2 + 1, !0) ? TYPE_RLI : TYPE_LRI),
                            (embedLevels[i$2] = stackTop._level),
                            stackTop._override && changeCharType(i$2, stackTop._override);
                          var level$2 = (charType === TYPE_RLI ? nextOdd : nextEven)(
                            stackTop._level,
                          );
                          level$2 <= 125 &&
                          0 === overflowIsolateCount &&
                          0 === overflowEmbeddingCount
                            ? (validIsolateCount++,
                              statusStack.push({
                                _level: level$2,
                                _override: 0,
                                _isolate: 1,
                                _isolInitIndex: i$2,
                              }))
                            : overflowIsolateCount++;
                        } else if (charType & TYPE_PDI) {
                          if (overflowIsolateCount > 0) overflowIsolateCount--;
                          else if (validIsolateCount > 0) {
                            for (
                              overflowEmbeddingCount = 0;
                              !statusStack[statusStack.length - 1]._isolate;

                            )
                              statusStack.pop();
                            var isolInitIndex = statusStack[statusStack.length - 1]._isolInitIndex;
                            null != isolInitIndex &&
                              (isolationPairs.set(isolInitIndex, i$2),
                              isolationPairs.set(i$2, isolInitIndex)),
                              statusStack.pop(),
                              validIsolateCount--;
                          }
                          (stackTop = statusStack[statusStack.length - 1]),
                            (embedLevels[i$2] = stackTop._level),
                            stackTop._override && changeCharType(i$2, stackTop._override);
                        } else
                          charType & TYPE_PDF
                            ? (0 === overflowIsolateCount &&
                                (overflowEmbeddingCount > 0
                                  ? overflowEmbeddingCount--
                                  : !stackTop._isolate &&
                                    statusStack.length > 1 &&
                                    (statusStack.pop(),
                                    (stackTop = statusStack[statusStack.length - 1]))),
                              (embedLevels[i$2] = stackTop._level))
                            : charType & TYPE_B && (embedLevels[i$2] = paragraph.level);
                      else
                        (embedLevels[i$2] = stackTop._level),
                          stackTop._override &&
                            charType !== TYPE_BN &&
                            changeCharType(i$2, stackTop._override);
                    }
                    for (
                      var levelRuns = [], currentRun = null, i$3 = paragraph.start;
                      i$3 <= paragraph.end;
                      i$3++
                    ) {
                      var charType$1 = charTypes[i$3];
                      if (!(charType$1 & BN_LIKE_TYPES)) {
                        var lvl = embedLevels[i$3],
                          isIsolInit = charType$1 & ISOLATE_INIT_TYPES,
                          isPDI = charType$1 === TYPE_PDI;
                        currentRun && lvl === currentRun._level
                          ? ((currentRun._end = i$3), (currentRun._endsWithIsolInit = isIsolInit))
                          : levelRuns.push(
                              (currentRun = {
                                _start: i$3,
                                _end: i$3,
                                _level: lvl,
                                _startsWithPDI: isPDI,
                                _endsWithIsolInit: isIsolInit,
                              }),
                            );
                      }
                    }
                    for (
                      var isolatingRunSeqs = [], runIdx = 0;
                      runIdx < levelRuns.length;
                      runIdx++
                    ) {
                      var run = levelRuns[runIdx];
                      if (
                        !run._startsWithPDI ||
                        (run._startsWithPDI && !isolationPairs.has(run._start))
                      ) {
                        for (
                          var seqRuns = [(currentRun = run)], pdiIndex = void 0;
                          currentRun &&
                          currentRun._endsWithIsolInit &&
                          null != (pdiIndex = isolationPairs.get(currentRun._end));

                        )
                          for (var i$4 = runIdx + 1; i$4 < levelRuns.length; i$4++)
                            if (levelRuns[i$4]._start === pdiIndex) {
                              seqRuns.push((currentRun = levelRuns[i$4]));
                              break;
                            }
                        for (var seqIndices = [], i$5 = 0; i$5 < seqRuns.length; i$5++)
                          for (var run$1 = seqRuns[i$5], j = run$1._start; j <= run$1._end; j++)
                            seqIndices.push(j);
                        for (
                          var firstLevel = embedLevels[seqIndices[0]],
                            prevLevel = paragraph.level,
                            i$6 = seqIndices[0] - 1;
                          i$6 >= 0;
                          i$6--
                        )
                          if (!(charTypes[i$6] & BN_LIKE_TYPES)) {
                            prevLevel = embedLevels[i$6];
                            break;
                          }
                        var lastIndex = seqIndices[seqIndices.length - 1],
                          lastLevel = embedLevels[lastIndex],
                          nextLevel = paragraph.level;
                        if (!(charTypes[lastIndex] & ISOLATE_INIT_TYPES))
                          for (var i$7 = lastIndex + 1; i$7 <= paragraph.end; i$7++)
                            if (!(charTypes[i$7] & BN_LIKE_TYPES)) {
                              nextLevel = embedLevels[i$7];
                              break;
                            }
                        isolatingRunSeqs.push({
                          _seqIndices: seqIndices,
                          _sosType: Math.max(prevLevel, firstLevel) % 2 ? TYPE_R : TYPE_L,
                          _eosType: Math.max(nextLevel, lastLevel) % 2 ? TYPE_R : TYPE_L,
                        });
                      }
                    }
                    for (var seqIdx = 0; seqIdx < isolatingRunSeqs.length; seqIdx++) {
                      var ref = isolatingRunSeqs[seqIdx],
                        seqIndices$1 = ref._seqIndices,
                        sosType = ref._sosType,
                        eosType = ref._eosType,
                        embedDirection = 1 & embedLevels[seqIndices$1[0]] ? TYPE_R : TYPE_L;
                      if (charTypeCounts.get(TYPE_NSM))
                        for (var si = 0; si < seqIndices$1.length; si++) {
                          var i$8 = seqIndices$1[si];
                          if (charTypes[i$8] & TYPE_NSM) {
                            for (var prevType = sosType, sj = si - 1; sj >= 0; sj--)
                              if (!(charTypes[seqIndices$1[sj]] & BN_LIKE_TYPES)) {
                                prevType = charTypes[seqIndices$1[sj]];
                                break;
                              }
                            changeCharType(
                              i$8,
                              prevType & (ISOLATE_INIT_TYPES | TYPE_PDI) ? TYPE_ON : prevType,
                            );
                          }
                        }
                      if (charTypeCounts.get(TYPE_EN))
                        for (var si$1 = 0; si$1 < seqIndices$1.length; si$1++) {
                          var i$9 = seqIndices$1[si$1];
                          if (charTypes[i$9] & TYPE_EN)
                            for (var sj$1 = si$1 - 1; sj$1 >= -1; sj$1--) {
                              var prevCharType =
                                -1 === sj$1 ? sosType : charTypes[seqIndices$1[sj$1]];
                              if (prevCharType & STRONG_TYPES) {
                                prevCharType === TYPE_AL && changeCharType(i$9, TYPE_AN);
                                break;
                              }
                            }
                        }
                      if (charTypeCounts.get(TYPE_AL))
                        for (var si$2 = 0; si$2 < seqIndices$1.length; si$2++) {
                          var i$10 = seqIndices$1[si$2];
                          charTypes[i$10] & TYPE_AL && changeCharType(i$10, TYPE_R);
                        }
                      if (charTypeCounts.get(TYPE_ES) || charTypeCounts.get(TYPE_CS))
                        for (var si$3 = 1; si$3 < seqIndices$1.length - 1; si$3++) {
                          var i$11 = seqIndices$1[si$3];
                          if (charTypes[i$11] & (TYPE_ES | TYPE_CS)) {
                            for (
                              var prevType$1 = 0, nextType = 0, sj$2 = si$3 - 1;
                              sj$2 >= 0 &&
                              (prevType$1 = charTypes[seqIndices$1[sj$2]]) & BN_LIKE_TYPES;
                              sj$2--
                            );
                            for (
                              var sj$3 = si$3 + 1;
                              sj$3 < seqIndices$1.length &&
                              (nextType = charTypes[seqIndices$1[sj$3]]) & BN_LIKE_TYPES;
                              sj$3++
                            );
                            prevType$1 === nextType &&
                              (charTypes[i$11] === TYPE_ES
                                ? prevType$1 === TYPE_EN
                                : prevType$1 & (TYPE_EN | TYPE_AN)) &&
                              changeCharType(i$11, prevType$1);
                          }
                        }
                      if (charTypeCounts.get(TYPE_EN))
                        for (var si$4 = 0; si$4 < seqIndices$1.length; si$4++) {
                          var i$12 = seqIndices$1[si$4];
                          if (charTypes[i$12] & TYPE_EN) {
                            for (
                              var sj$4 = si$4 - 1;
                              sj$4 >= 0 &&
                              charTypes[seqIndices$1[sj$4]] & (TYPE_ET | BN_LIKE_TYPES);
                              sj$4--
                            )
                              changeCharType(seqIndices$1[sj$4], TYPE_EN);
                            for (
                              si$4++;
                              si$4 < seqIndices$1.length &&
                              charTypes[seqIndices$1[si$4]] & (TYPE_ET | BN_LIKE_TYPES | TYPE_EN);
                              si$4++
                            )
                              charTypes[seqIndices$1[si$4]] !== TYPE_EN &&
                                changeCharType(seqIndices$1[si$4], TYPE_EN);
                          }
                        }
                      if (
                        charTypeCounts.get(TYPE_ET) ||
                        charTypeCounts.get(TYPE_ES) ||
                        charTypeCounts.get(TYPE_CS)
                      )
                        for (var si$5 = 0; si$5 < seqIndices$1.length; si$5++) {
                          var i$13 = seqIndices$1[si$5];
                          if (charTypes[i$13] & (TYPE_ET | TYPE_ES | TYPE_CS)) {
                            changeCharType(i$13, TYPE_ON);
                            for (
                              var sj$5 = si$5 - 1;
                              sj$5 >= 0 && charTypes[seqIndices$1[sj$5]] & BN_LIKE_TYPES;
                              sj$5--
                            )
                              changeCharType(seqIndices$1[sj$5], TYPE_ON);
                            for (
                              var sj$6 = si$5 + 1;
                              sj$6 < seqIndices$1.length &&
                              charTypes[seqIndices$1[sj$6]] & BN_LIKE_TYPES;
                              sj$6++
                            )
                              changeCharType(seqIndices$1[sj$6], TYPE_ON);
                          }
                        }
                      if (charTypeCounts.get(TYPE_EN))
                        for (
                          var si$6 = 0, prevStrongType = sosType;
                          si$6 < seqIndices$1.length;
                          si$6++
                        ) {
                          var i$14 = seqIndices$1[si$6],
                            type = charTypes[i$14];
                          type & TYPE_EN
                            ? prevStrongType === TYPE_L && changeCharType(i$14, TYPE_L)
                            : type & STRONG_TYPES && (prevStrongType = type);
                        }
                      if (charTypeCounts.get(NEUTRAL_ISOLATE_TYPES)) {
                        for (
                          var R_TYPES_FOR_N_STEPS = TYPE_R | TYPE_EN | TYPE_AN,
                            STRONG_TYPES_FOR_N_STEPS = R_TYPES_FOR_N_STEPS | TYPE_L,
                            bracketPairs = [],
                            openerStack = [],
                            si$7 = 0;
                          si$7 < seqIndices$1.length;
                          si$7++
                        )
                          if (charTypes[seqIndices$1[si$7]] & NEUTRAL_ISOLATE_TYPES) {
                            var char = string[seqIndices$1[si$7]],
                              oppositeBracket = void 0;
                            if (null !== openingToClosingBracket(char)) {
                              if (!(openerStack.length < 63)) break;
                              openerStack.push({ char, seqIndex: si$7 });
                            } else if (null !== (oppositeBracket = closingToOpeningBracket(char)))
                              for (
                                var stackIdx = openerStack.length - 1;
                                stackIdx >= 0;
                                stackIdx--
                              ) {
                                var stackChar = openerStack[stackIdx].char;
                                if (
                                  stackChar === oppositeBracket ||
                                  stackChar ===
                                    closingToOpeningBracket(getCanonicalBracket(char)) ||
                                  openingToClosingBracket(getCanonicalBracket(stackChar)) === char
                                ) {
                                  bracketPairs.push([openerStack[stackIdx].seqIndex, si$7]),
                                    (openerStack.length = stackIdx);
                                  break;
                                }
                              }
                          }
                        bracketPairs.sort(function (a, b) {
                          return a[0] - b[0];
                        });
                        for (var pairIdx = 0; pairIdx < bracketPairs.length; pairIdx++) {
                          for (
                            var ref$1 = bracketPairs[pairIdx],
                              openSeqIdx = ref$1[0],
                              closeSeqIdx = ref$1[1],
                              foundStrongType = !1,
                              useStrongType = 0,
                              si$8 = openSeqIdx + 1;
                            si$8 < closeSeqIdx;
                            si$8++
                          ) {
                            var i$15 = seqIndices$1[si$8];
                            if (charTypes[i$15] & STRONG_TYPES_FOR_N_STEPS) {
                              foundStrongType = !0;
                              var lr = charTypes[i$15] & R_TYPES_FOR_N_STEPS ? TYPE_R : TYPE_L;
                              if (lr === embedDirection) {
                                useStrongType = lr;
                                break;
                              }
                            }
                          }
                          if (foundStrongType && !useStrongType) {
                            useStrongType = sosType;
                            for (var si$9 = openSeqIdx - 1; si$9 >= 0; si$9--) {
                              var i$16 = seqIndices$1[si$9];
                              if (charTypes[i$16] & STRONG_TYPES_FOR_N_STEPS) {
                                var lr$1 = charTypes[i$16] & R_TYPES_FOR_N_STEPS ? TYPE_R : TYPE_L;
                                useStrongType = lr$1 !== embedDirection ? lr$1 : embedDirection;
                                break;
                              }
                            }
                          }
                          if (useStrongType) {
                            if (
                              ((charTypes[seqIndices$1[openSeqIdx]] = charTypes[
                                seqIndices$1[closeSeqIdx]
                              ] =
                                useStrongType),
                              useStrongType !== embedDirection)
                            )
                              for (var si$10 = openSeqIdx + 1; si$10 < seqIndices$1.length; si$10++)
                                if (!(charTypes[seqIndices$1[si$10]] & BN_LIKE_TYPES)) {
                                  getBidiCharType(string[seqIndices$1[si$10]]) & TYPE_NSM &&
                                    (charTypes[seqIndices$1[si$10]] = useStrongType);
                                  break;
                                }
                            if (useStrongType !== embedDirection)
                              for (
                                var si$11 = closeSeqIdx + 1;
                                si$11 < seqIndices$1.length;
                                si$11++
                              )
                                if (!(charTypes[seqIndices$1[si$11]] & BN_LIKE_TYPES)) {
                                  getBidiCharType(string[seqIndices$1[si$11]]) & TYPE_NSM &&
                                    (charTypes[seqIndices$1[si$11]] = useStrongType);
                                  break;
                                }
                          }
                        }
                        for (var si$12 = 0; si$12 < seqIndices$1.length; si$12++)
                          if (charTypes[seqIndices$1[si$12]] & NEUTRAL_ISOLATE_TYPES) {
                            for (
                              var niRunStart = si$12,
                                niRunEnd = si$12,
                                prevType$2 = sosType,
                                si2 = si$12 - 1;
                              si2 >= 0;
                              si2--
                            ) {
                              if (!(charTypes[seqIndices$1[si2]] & BN_LIKE_TYPES)) {
                                prevType$2 =
                                  charTypes[seqIndices$1[si2]] & R_TYPES_FOR_N_STEPS
                                    ? TYPE_R
                                    : TYPE_L;
                                break;
                              }
                              niRunStart = si2;
                            }
                            for (
                              var nextType$1 = eosType, si2$1 = si$12 + 1;
                              si2$1 < seqIndices$1.length;
                              si2$1++
                            ) {
                              if (
                                !(
                                  charTypes[seqIndices$1[si2$1]] &
                                  (NEUTRAL_ISOLATE_TYPES | BN_LIKE_TYPES)
                                )
                              ) {
                                nextType$1 =
                                  charTypes[seqIndices$1[si2$1]] & R_TYPES_FOR_N_STEPS
                                    ? TYPE_R
                                    : TYPE_L;
                                break;
                              }
                              niRunEnd = si2$1;
                            }
                            for (var sj$7 = niRunStart; sj$7 <= niRunEnd; sj$7++)
                              charTypes[seqIndices$1[sj$7]] =
                                prevType$2 === nextType$1 ? prevType$2 : embedDirection;
                            si$12 = niRunEnd;
                          }
                      }
                    }
                    for (var i$17 = paragraph.start; i$17 <= paragraph.end; i$17++) {
                      var level$3 = embedLevels[i$17],
                        type$1 = charTypes[i$17];
                      if (
                        (1 & level$3
                          ? type$1 & (TYPE_L | TYPE_EN | TYPE_AN) && embedLevels[i$17]++
                          : type$1 & TYPE_R
                            ? embedLevels[i$17]++
                            : type$1 & (TYPE_AN | TYPE_EN) && (embedLevels[i$17] += 2),
                        type$1 & BN_LIKE_TYPES &&
                          (embedLevels[i$17] =
                            0 === i$17 ? paragraph.level : embedLevels[i$17 - 1]),
                        i$17 === paragraph.end || getBidiCharType(string[i$17]) & (TYPE_S | TYPE_B))
                      )
                        for (
                          var j$1 = i$17;
                          j$1 >= 0 && getBidiCharType(string[j$1]) & TRAILING_TYPES;
                          j$1--
                        )
                          embedLevels[j$1] = paragraph.level;
                    }
                  }
                  return { levels: embedLevels, paragraphs };
                  function determineAutoEmbedLevel(start, isFSI) {
                    for (var i = start; i < string.length; i++) {
                      var charType = charTypes[i];
                      if (charType & (TYPE_R | TYPE_AL)) return 1;
                      if (charType & (TYPE_B | TYPE_L) || (isFSI && charType === TYPE_PDI))
                        return 0;
                      if (charType & ISOLATE_INIT_TYPES) {
                        var pdi = indexOfMatchingPDI(i);
                        i = -1 === pdi ? string.length : pdi;
                      }
                    }
                    return 0;
                  }
                  function indexOfMatchingPDI(isolateStart) {
                    for (var isolationLevel = 1, i = isolateStart + 1; i < string.length; i++) {
                      var charType = charTypes[i];
                      if (charType & TYPE_B) break;
                      if (charType & TYPE_PDI) {
                        if (0 == --isolationLevel) return i;
                      } else charType & ISOLATE_INIT_TYPES && isolationLevel++;
                    }
                    return -1;
                  }
                }),
                (exports.getMirroredCharacter = getMirroredCharacter),
                (exports.getMirroredCharactersMap = function getMirroredCharactersMap(
                  string,
                  embeddingLevels,
                  start,
                  end,
                ) {
                  var strLen = string.length;
                  (start = Math.max(0, null == start ? 0 : +start)),
                    (end = Math.min(strLen - 1, null == end ? strLen - 1 : +end));
                  for (var map = new Map(), i = start; i <= end; i++)
                    if (1 & embeddingLevels[i]) {
                      var mirror = getMirroredCharacter(string[i]);
                      null !== mirror && map.set(i, mirror);
                    }
                  return map;
                }),
                (exports.getReorderSegments = getReorderSegments),
                (exports.getReorderedIndices = getReorderedIndices),
                (exports.getReorderedString = function getReorderedString(
                  string,
                  embedLevelsResult,
                  start,
                  end,
                ) {
                  var indices = getReorderedIndices(string, embedLevelsResult, start, end),
                    chars = [].concat(string);
                  return (
                    indices.forEach(function (charIndex, i) {
                      chars[i] =
                        (1 & embedLevelsResult.levels[charIndex]
                          ? getMirroredCharacter(string[charIndex])
                          : null) || string[charIndex];
                    }),
                    chars.join('')
                  );
                }),
                (exports.openingToClosingBracket = openingToClosingBracket),
                Object.defineProperty(exports, '__esModule', { value: !0 }),
                exports
              );
            })({});
          },
          voidMainRegExp = /\bvoid\s+main\s*\(\s*\)\s*{/g;
        function expandShaderIncludes(source) {
          return source.replace(
            /^[ \t]*#include +<([\w\d./]+)>/gm,
            function replace(match, include) {
              let chunk = three_module.vxI[include];
              return chunk ? expandShaderIncludes(chunk) : match;
            },
          );
        }
        const _lut = [];
        for (let i = 0; i < 256; i++) _lut[i] = (i < 16 ? '0' : '') + i.toString(16);
        const troika_three_utils_esm_assign =
            Object.assign ||
            function () {
              let target = arguments[0];
              for (let i = 1, len = arguments.length; i < len; i++) {
                let source = arguments[i];
                if (source)
                  for (let prop in source)
                    Object.prototype.hasOwnProperty.call(source, prop) &&
                      (target[prop] = source[prop]);
              }
              return target;
            },
          epoch = Date.now(),
          CONSTRUCTOR_CACHE = new WeakMap(),
          SHADER_UPGRADE_CACHE = new Map();
        let materialInstanceId = 1e10;
        function createDerivedMaterial(baseMaterial, options) {
          const optionsKey = (function getKeyForOptions(options) {
            const optionsHash = JSON.stringify(options, optionsJsonReplacer);
            let id = optionsHashesToIds.get(optionsHash);
            null == id && optionsHashesToIds.set(optionsHash, (id = ++_idCtr));
            return id;
          })(options);
          let ctorsByDerivation = CONSTRUCTOR_CACHE.get(baseMaterial);
          if (
            (ctorsByDerivation ||
              CONSTRUCTOR_CACHE.set(baseMaterial, (ctorsByDerivation = Object.create(null))),
            ctorsByDerivation[optionsKey])
          )
            return new ctorsByDerivation[optionsKey]();
          const privateBeforeCompileProp = `_onBeforeCompile${optionsKey}`,
            onBeforeCompile = function (shaderInfo, renderer) {
              baseMaterial.onBeforeCompile.call(this, shaderInfo, renderer);
              const cacheKey =
                this.customProgramCacheKey() +
                '|' +
                shaderInfo.vertexShader +
                '|' +
                shaderInfo.fragmentShader;
              let upgradedShaders = SHADER_UPGRADE_CACHE[cacheKey];
              if (!upgradedShaders) {
                const upgraded = (function upgradeShaders(
                  material,
                  { vertexShader, fragmentShader },
                  options,
                  key,
                ) {
                  let {
                    vertexDefs,
                    vertexMainIntro,
                    vertexMainOutro,
                    vertexTransform,
                    fragmentDefs,
                    fragmentMainIntro,
                    fragmentMainOutro,
                    fragmentColorTransform,
                    customRewriter,
                    timeUniform,
                  } = options;
                  (vertexDefs = vertexDefs || ''),
                    (vertexMainIntro = vertexMainIntro || ''),
                    (vertexMainOutro = vertexMainOutro || ''),
                    (fragmentDefs = fragmentDefs || ''),
                    (fragmentMainIntro = fragmentMainIntro || ''),
                    (fragmentMainOutro = fragmentMainOutro || ''),
                    (vertexTransform || customRewriter) &&
                      (vertexShader = expandShaderIncludes(vertexShader));
                  (fragmentColorTransform || customRewriter) &&
                    (fragmentShader = expandShaderIncludes(
                      (fragmentShader = fragmentShader.replace(
                        /^[ \t]*#include <((?:tonemapping|encodings|fog|premultiplied_alpha|dithering)_fragment)>/gm,
                        '\n//!BEGIN_POST_CHUNK $1\n$&\n//!END_POST_CHUNK\n',
                      )),
                    ));
                  if (customRewriter) {
                    let res = customRewriter({ vertexShader, fragmentShader });
                    (vertexShader = res.vertexShader), (fragmentShader = res.fragmentShader);
                  }
                  if (fragmentColorTransform) {
                    let postChunks = [];
                    (fragmentShader = fragmentShader.replace(
                      /^\/\/!BEGIN_POST_CHUNK[^]+?^\/\/!END_POST_CHUNK/gm,
                      (match) => (postChunks.push(match), ''),
                    )),
                      (fragmentMainOutro = `${fragmentColorTransform}\n${postChunks.join('\n')}\n${fragmentMainOutro}`);
                  }
                  if (timeUniform) {
                    const code = `\nuniform float ${timeUniform};\n`;
                    (vertexDefs = code + vertexDefs), (fragmentDefs = code + fragmentDefs);
                  }
                  vertexTransform &&
                    ((vertexDefs = `${vertexDefs}\nvoid troikaVertexTransform${key}(inout vec3 position, inout vec3 normal, inout vec2 uv) {\n  ${vertexTransform}\n}\n`),
                    (vertexMainIntro = `\ntroika_position_${key} = vec3(position);\ntroika_normal_${key} = vec3(normal);\ntroika_uv_${key} = vec2(uv);\ntroikaVertexTransform${key}(troika_position_${key}, troika_normal_${key}, troika_uv_${key});\n${vertexMainIntro}\n`),
                    (vertexShader =
                      (vertexShader = `vec3 troika_position_${key};\nvec3 troika_normal_${key};\nvec2 troika_uv_${key};\n${vertexShader}\n`).replace(
                        /\b(position|normal|uv)\b/g,
                        (match, match1, index, fullStr) =>
                          /\battribute\s+vec[23]\s+$/.test(fullStr.substr(0, index))
                            ? match1
                            : `troika_${match1}_${key}`,
                      )),
                    (material.map && material.map.channel > 0) ||
                      (vertexShader = vertexShader.replace(/\bMAP_UV\b/g, `troika_uv_${key}`)));
                  return (
                    (vertexShader = injectIntoShaderCode(
                      vertexShader,
                      key,
                      vertexDefs,
                      vertexMainIntro,
                      vertexMainOutro,
                    )),
                    (fragmentShader = injectIntoShaderCode(
                      fragmentShader,
                      key,
                      fragmentDefs,
                      fragmentMainIntro,
                      fragmentMainOutro,
                    )),
                    { vertexShader, fragmentShader }
                  );
                })(this, shaderInfo, options, optionsKey);
                upgradedShaders = SHADER_UPGRADE_CACHE[cacheKey] = upgraded;
              }
              (shaderInfo.vertexShader = upgradedShaders.vertexShader),
                (shaderInfo.fragmentShader = upgradedShaders.fragmentShader),
                troika_three_utils_esm_assign(shaderInfo.uniforms, this.uniforms),
                options.timeUniform &&
                  (shaderInfo.uniforms[options.timeUniform] = {
                    get value() {
                      return Date.now() - epoch;
                    },
                  }),
                this[privateBeforeCompileProp] && this[privateBeforeCompileProp](shaderInfo);
            },
            DerivedMaterial = function DerivedMaterial() {
              return derive(options.chained ? baseMaterial : baseMaterial.clone());
            },
            derive = function (base) {
              const derived = Object.create(base, descriptor);
              return (
                Object.defineProperty(derived, 'baseMaterial', { value: baseMaterial }),
                Object.defineProperty(derived, 'id', { value: materialInstanceId++ }),
                (derived.uuid = (function generateUUID() {
                  const d0 = (4294967295 * Math.random()) | 0,
                    d1 = (4294967295 * Math.random()) | 0,
                    d2 = (4294967295 * Math.random()) | 0,
                    d3 = (4294967295 * Math.random()) | 0;
                  return (
                    _lut[255 & d0] +
                    _lut[(d0 >> 8) & 255] +
                    _lut[(d0 >> 16) & 255] +
                    _lut[(d0 >> 24) & 255] +
                    '-' +
                    _lut[255 & d1] +
                    _lut[(d1 >> 8) & 255] +
                    '-' +
                    _lut[((d1 >> 16) & 15) | 64] +
                    _lut[(d1 >> 24) & 255] +
                    '-' +
                    _lut[(63 & d2) | 128] +
                    _lut[(d2 >> 8) & 255] +
                    '-' +
                    _lut[(d2 >> 16) & 255] +
                    _lut[(d2 >> 24) & 255] +
                    _lut[255 & d3] +
                    _lut[(d3 >> 8) & 255] +
                    _lut[(d3 >> 16) & 255] +
                    _lut[(d3 >> 24) & 255]
                  ).toUpperCase();
                })()),
                (derived.uniforms = troika_three_utils_esm_assign(
                  {},
                  base.uniforms,
                  options.uniforms,
                )),
                (derived.defines = troika_three_utils_esm_assign(
                  {},
                  base.defines,
                  options.defines,
                )),
                (derived.defines[`TROIKA_DERIVED_MATERIAL_${optionsKey}`] = ''),
                (derived.extensions = troika_three_utils_esm_assign(
                  {},
                  base.extensions,
                  options.extensions,
                )),
                (derived._listeners = void 0),
                derived
              );
            },
            descriptor = {
              constructor: { value: DerivedMaterial },
              isDerivedMaterial: { value: !0 },
              customProgramCacheKey: {
                writable: !0,
                configurable: !0,
                value: function () {
                  return baseMaterial.customProgramCacheKey() + '|' + optionsKey;
                },
              },
              onBeforeCompile: {
                get: () => onBeforeCompile,
                set(fn) {
                  this[privateBeforeCompileProp] = fn;
                },
              },
              copy: {
                writable: !0,
                configurable: !0,
                value: function (source) {
                  return (
                    baseMaterial.copy.call(this, source),
                    baseMaterial.isShaderMaterial ||
                      baseMaterial.isDerivedMaterial ||
                      (troika_three_utils_esm_assign(this.extensions, source.extensions),
                      troika_three_utils_esm_assign(this.defines, source.defines),
                      troika_three_utils_esm_assign(
                        this.uniforms,
                        three_module.LlO.clone(source.uniforms),
                      )),
                    this
                  );
                },
              },
              clone: {
                writable: !0,
                configurable: !0,
                value: function () {
                  const newBase = new baseMaterial.constructor();
                  return derive(newBase).copy(this);
                },
              },
              getDepthMaterial: {
                writable: !0,
                configurable: !0,
                value: function () {
                  let depthMaterial = this._depthMaterial;
                  return (
                    depthMaterial ||
                      ((depthMaterial = this._depthMaterial =
                        createDerivedMaterial(
                          baseMaterial.isDerivedMaterial
                            ? baseMaterial.getDepthMaterial()
                            : new three_module.CSG({ depthPacking: three_module.N5j }),
                          options,
                        )),
                      (depthMaterial.defines.IS_DEPTH_MATERIAL = ''),
                      (depthMaterial.uniforms = this.uniforms)),
                    depthMaterial
                  );
                },
              },
              getDistanceMaterial: {
                writable: !0,
                configurable: !0,
                value: function () {
                  let distanceMaterial = this._distanceMaterial;
                  return (
                    distanceMaterial ||
                      ((distanceMaterial = this._distanceMaterial =
                        createDerivedMaterial(
                          baseMaterial.isDerivedMaterial
                            ? baseMaterial.getDistanceMaterial()
                            : new three_module.aVO(),
                          options,
                        )),
                      (distanceMaterial.defines.IS_DISTANCE_MATERIAL = ''),
                      (distanceMaterial.uniforms = this.uniforms)),
                    distanceMaterial
                  );
                },
              },
              dispose: {
                writable: !0,
                configurable: !0,
                value() {
                  const { _depthMaterial, _distanceMaterial } = this;
                  _depthMaterial && _depthMaterial.dispose(),
                    _distanceMaterial && _distanceMaterial.dispose(),
                    baseMaterial.dispose.call(this);
                },
              },
            };
          return (ctorsByDerivation[optionsKey] = DerivedMaterial), new DerivedMaterial();
        }
        function injectIntoShaderCode(shaderCode, id, defs, intro, outro) {
          return (
            (intro || outro || defs) &&
              ((shaderCode = shaderCode.replace(
                voidMainRegExp,
                `\n${defs}\nvoid troikaOrigMain${id}() {`,
              )),
              (shaderCode += `\nvoid main() {\n  ${intro}\n  troikaOrigMain${id}();\n  ${outro}\n}`)),
            shaderCode
          );
        }
        function optionsJsonReplacer(key, value) {
          return 'uniforms' === key
            ? void 0
            : 'function' == typeof value
              ? value.toString()
              : value;
        }
        let _idCtr = 0;
        const optionsHashesToIds = new Map();
        three_module.$EB;
        three_module.eaF;
        const workerModule = defineWorkerModule({
          name: 'Typr Font Parser',
          dependencies: [
            function typrFactory() {
              return (
                'undefined' == typeof window && (self.window = self),
                (function (r) {
                  var e = {
                    parse: function (r) {
                      var t = e._bin,
                        a = new Uint8Array(r);
                      if ('ttcf' == t.readASCII(a, 0, 4)) {
                        var n = 4;
                        t.readUshort(a, n), (n += 2), t.readUshort(a, n), (n += 2);
                        var o = t.readUint(a, n);
                        n += 4;
                        for (var s = [], i = 0; i < o; i++) {
                          var h = t.readUint(a, n);
                          (n += 4), s.push(e._readFont(a, h));
                        }
                        return s;
                      }
                      return [e._readFont(a, 0)];
                    },
                    _readFont: function (r, t) {
                      var a = e._bin,
                        n = t;
                      a.readFixed(r, t), (t += 4);
                      var o = a.readUshort(r, t);
                      (t += 2),
                        a.readUshort(r, t),
                        (t += 2),
                        a.readUshort(r, t),
                        (t += 2),
                        a.readUshort(r, t),
                        (t += 2);
                      for (
                        var s = [
                            'cmap',
                            'head',
                            'hhea',
                            'maxp',
                            'hmtx',
                            'name',
                            'OS/2',
                            'post',
                            'loca',
                            'glyf',
                            'kern',
                            'CFF ',
                            'GDEF',
                            'GPOS',
                            'GSUB',
                            'SVG ',
                          ],
                          i = { _data: r, _offset: n },
                          h = {},
                          d = 0;
                        d < o;
                        d++
                      ) {
                        var f = a.readASCII(r, t, 4);
                        (t += 4), a.readUint(r, t), (t += 4);
                        var u = a.readUint(r, t);
                        t += 4;
                        var l = a.readUint(r, t);
                        (t += 4), (h[f] = { offset: u, length: l });
                      }
                      for (d = 0; d < s.length; d++) {
                        var v = s[d];
                        h[v] && (i[v.trim()] = e[v.trim()].parse(r, h[v].offset, h[v].length, i));
                      }
                      return i;
                    },
                    _tabOffset: function (r, t, a) {
                      for (
                        var n = e._bin, o = n.readUshort(r, a + 4), s = a + 12, i = 0;
                        i < o;
                        i++
                      ) {
                        var h = n.readASCII(r, s, 4);
                        (s += 4), n.readUint(r, s), (s += 4);
                        var d = n.readUint(r, s);
                        if (((s += 4), n.readUint(r, s), (s += 4), h == t)) return d;
                      }
                      return 0;
                    },
                  };
                  (e._bin = {
                    readFixed: function (r, e) {
                      return ((r[e] << 8) | r[e + 1]) + ((r[e + 2] << 8) | r[e + 3]) / 65540;
                    },
                    readF2dot14: function (r, t) {
                      return e._bin.readShort(r, t) / 16384;
                    },
                    readInt: function (r, t) {
                      return e._bin._view(r).getInt32(t);
                    },
                    readInt8: function (r, t) {
                      return e._bin._view(r).getInt8(t);
                    },
                    readShort: function (r, t) {
                      return e._bin._view(r).getInt16(t);
                    },
                    readUshort: function (r, t) {
                      return e._bin._view(r).getUint16(t);
                    },
                    readUshorts: function (r, t, a) {
                      for (var n = [], o = 0; o < a; o++) n.push(e._bin.readUshort(r, t + 2 * o));
                      return n;
                    },
                    readUint: function (r, t) {
                      return e._bin._view(r).getUint32(t);
                    },
                    readUint64: function (r, t) {
                      return 4294967296 * e._bin.readUint(r, t) + e._bin.readUint(r, t + 4);
                    },
                    readASCII: function (r, e, t) {
                      for (var a = '', n = 0; n < t; n++) a += String.fromCharCode(r[e + n]);
                      return a;
                    },
                    readUnicode: function (r, e, t) {
                      for (var a = '', n = 0; n < t; n++) {
                        var o = (r[e++] << 8) | r[e++];
                        a += String.fromCharCode(o);
                      }
                      return a;
                    },
                    _tdec:
                      'undefined' != typeof window && window.TextDecoder
                        ? new window.TextDecoder()
                        : null,
                    readUTF8: function (r, t, a) {
                      var n = e._bin._tdec;
                      return n && 0 == t && a == r.length ? n.decode(r) : e._bin.readASCII(r, t, a);
                    },
                    readBytes: function (r, e, t) {
                      for (var a = [], n = 0; n < t; n++) a.push(r[e + n]);
                      return a;
                    },
                    readASCIIArray: function (r, e, t) {
                      for (var a = [], n = 0; n < t; n++) a.push(String.fromCharCode(r[e + n]));
                      return a;
                    },
                    _view: function (r) {
                      return (
                        r._dataView ||
                        (r._dataView = r.buffer
                          ? new DataView(r.buffer, r.byteOffset, r.byteLength)
                          : new DataView(new Uint8Array(r).buffer))
                      );
                    },
                  }),
                    (e._lctf = {}),
                    (e._lctf.parse = function (r, t, a, n, o) {
                      var s = e._bin,
                        i = {},
                        h = t;
                      s.readFixed(r, t), (t += 4);
                      var d = s.readUshort(r, t);
                      t += 2;
                      var f = s.readUshort(r, t);
                      t += 2;
                      var u = s.readUshort(r, t);
                      return (
                        (t += 2),
                        (i.scriptList = e._lctf.readScriptList(r, h + d)),
                        (i.featureList = e._lctf.readFeatureList(r, h + f)),
                        (i.lookupList = e._lctf.readLookupList(r, h + u, o)),
                        i
                      );
                    }),
                    (e._lctf.readLookupList = function (r, t, a) {
                      var n = e._bin,
                        o = t,
                        s = [],
                        i = n.readUshort(r, t);
                      t += 2;
                      for (var h = 0; h < i; h++) {
                        var d = n.readUshort(r, t);
                        t += 2;
                        var f = e._lctf.readLookupTable(r, o + d, a);
                        s.push(f);
                      }
                      return s;
                    }),
                    (e._lctf.readLookupTable = function (r, t, a) {
                      var n = e._bin,
                        o = t,
                        s = { tabs: [] };
                      (s.ltype = n.readUshort(r, t)),
                        (t += 2),
                        (s.flag = n.readUshort(r, t)),
                        (t += 2);
                      var i = n.readUshort(r, t);
                      t += 2;
                      for (var h = s.ltype, d = 0; d < i; d++) {
                        var f = n.readUshort(r, t);
                        t += 2;
                        var u = a(r, h, o + f, s);
                        s.tabs.push(u);
                      }
                      return s;
                    }),
                    (e._lctf.numOfOnes = function (r) {
                      for (var e = 0, t = 0; t < 32; t++) (r >>> t) & 1 && e++;
                      return e;
                    }),
                    (e._lctf.readClassDef = function (r, t) {
                      var a = e._bin,
                        n = [],
                        o = a.readUshort(r, t);
                      if (((t += 2), 1 == o)) {
                        var s = a.readUshort(r, t);
                        t += 2;
                        var i = a.readUshort(r, t);
                        t += 2;
                        for (var h = 0; h < i; h++)
                          n.push(s + h), n.push(s + h), n.push(a.readUshort(r, t)), (t += 2);
                      }
                      if (2 == o) {
                        var d = a.readUshort(r, t);
                        for (t += 2, h = 0; h < d; h++)
                          n.push(a.readUshort(r, t)),
                            (t += 2),
                            n.push(a.readUshort(r, t)),
                            (t += 2),
                            n.push(a.readUshort(r, t)),
                            (t += 2);
                      }
                      return n;
                    }),
                    (e._lctf.getInterval = function (r, e) {
                      for (var t = 0; t < r.length; t += 3) {
                        var a = r[t],
                          n = r[t + 1];
                        if ((r[t + 2], a <= e && e <= n)) return t;
                      }
                      return -1;
                    }),
                    (e._lctf.readCoverage = function (r, t) {
                      var a = e._bin,
                        n = {};
                      (n.fmt = a.readUshort(r, t)), (t += 2);
                      var o = a.readUshort(r, t);
                      return (
                        (t += 2),
                        1 == n.fmt && (n.tab = a.readUshorts(r, t, o)),
                        2 == n.fmt && (n.tab = a.readUshorts(r, t, 3 * o)),
                        n
                      );
                    }),
                    (e._lctf.coverageIndex = function (r, t) {
                      var a = r.tab;
                      if (1 == r.fmt) return a.indexOf(t);
                      if (2 == r.fmt) {
                        var n = e._lctf.getInterval(a, t);
                        if (-1 != n) return a[n + 2] + (t - a[n]);
                      }
                      return -1;
                    }),
                    (e._lctf.readFeatureList = function (r, t) {
                      var a = e._bin,
                        n = t,
                        o = [],
                        s = a.readUshort(r, t);
                      t += 2;
                      for (var i = 0; i < s; i++) {
                        var h = a.readASCII(r, t, 4);
                        t += 4;
                        var d = a.readUshort(r, t);
                        t += 2;
                        var f = e._lctf.readFeatureTable(r, n + d);
                        (f.tag = h.trim()), o.push(f);
                      }
                      return o;
                    }),
                    (e._lctf.readFeatureTable = function (r, t) {
                      var a = e._bin,
                        n = t,
                        o = {},
                        s = a.readUshort(r, t);
                      (t += 2), s > 0 && (o.featureParams = n + s);
                      var i = a.readUshort(r, t);
                      (t += 2), (o.tab = []);
                      for (var h = 0; h < i; h++) o.tab.push(a.readUshort(r, t + 2 * h));
                      return o;
                    }),
                    (e._lctf.readScriptList = function (r, t) {
                      var a = e._bin,
                        n = t,
                        o = {},
                        s = a.readUshort(r, t);
                      t += 2;
                      for (var i = 0; i < s; i++) {
                        var h = a.readASCII(r, t, 4);
                        t += 4;
                        var d = a.readUshort(r, t);
                        (t += 2), (o[h.trim()] = e._lctf.readScriptTable(r, n + d));
                      }
                      return o;
                    }),
                    (e._lctf.readScriptTable = function (r, t) {
                      var a = e._bin,
                        n = t,
                        o = {},
                        s = a.readUshort(r, t);
                      (t += 2), s > 0 && (o.default = e._lctf.readLangSysTable(r, n + s));
                      var i = a.readUshort(r, t);
                      t += 2;
                      for (var h = 0; h < i; h++) {
                        var d = a.readASCII(r, t, 4);
                        t += 4;
                        var f = a.readUshort(r, t);
                        (t += 2), (o[d.trim()] = e._lctf.readLangSysTable(r, n + f));
                      }
                      return o;
                    }),
                    (e._lctf.readLangSysTable = function (r, t) {
                      var a = e._bin,
                        n = {};
                      a.readUshort(r, t), (t += 2), (n.reqFeature = a.readUshort(r, t)), (t += 2);
                      var o = a.readUshort(r, t);
                      return (t += 2), (n.features = a.readUshorts(r, t, o)), n;
                    }),
                    (e.CFF = {}),
                    (e.CFF.parse = function (r, t, a) {
                      var n = e._bin;
                      (r = new Uint8Array(r.buffer, t, a))[(t = 0)], r[++t], r[++t], r[++t], t++;
                      var o = [];
                      t = e.CFF.readIndex(r, t, o);
                      for (var s = [], i = 0; i < o.length - 1; i++)
                        s.push(n.readASCII(r, t + o[i], o[i + 1] - o[i]));
                      t += o[o.length - 1];
                      var h = [];
                      t = e.CFF.readIndex(r, t, h);
                      var d = [];
                      for (i = 0; i < h.length - 1; i++)
                        d.push(e.CFF.readDict(r, t + h[i], t + h[i + 1]));
                      t += h[h.length - 1];
                      var f = d[0],
                        u = [];
                      t = e.CFF.readIndex(r, t, u);
                      var l = [];
                      for (i = 0; i < u.length - 1; i++)
                        l.push(n.readASCII(r, t + u[i], u[i + 1] - u[i]));
                      if (((t += u[u.length - 1]), e.CFF.readSubrs(r, t, f), f.CharStrings)) {
                        (t = f.CharStrings), (u = []), (t = e.CFF.readIndex(r, t, u));
                        var v = [];
                        for (i = 0; i < u.length - 1; i++)
                          v.push(n.readBytes(r, t + u[i], u[i + 1] - u[i]));
                        f.CharStrings = v;
                      }
                      if (f.ROS) {
                        t = f.FDArray;
                        var c = [];
                        for (
                          t = e.CFF.readIndex(r, t, c), f.FDArray = [], i = 0;
                          i < c.length - 1;
                          i++
                        ) {
                          var p = e.CFF.readDict(r, t + c[i], t + c[i + 1]);
                          e.CFF._readFDict(r, p, l), f.FDArray.push(p);
                        }
                        (t += c[c.length - 1]), (t = f.FDSelect), (f.FDSelect = []);
                        var U = r[t];
                        if ((t++, 3 != U)) throw U;
                        var g = n.readUshort(r, t);
                        for (t += 2, i = 0; i < g + 1; i++)
                          f.FDSelect.push(n.readUshort(r, t), r[t + 2]), (t += 3);
                      }
                      return (
                        f.Encoding &&
                          (f.Encoding = e.CFF.readEncoding(r, f.Encoding, f.CharStrings.length)),
                        f.charset &&
                          (f.charset = e.CFF.readCharset(r, f.charset, f.CharStrings.length)),
                        e.CFF._readFDict(r, f, l),
                        f
                      );
                    }),
                    (e.CFF._readFDict = function (r, t, a) {
                      var n;
                      for (var o in (t.Private &&
                        ((n = t.Private[1]),
                        (t.Private = e.CFF.readDict(r, n, n + t.Private[0])),
                        t.Private.Subrs && e.CFF.readSubrs(r, n + t.Private.Subrs, t.Private)),
                      t))
                        -1 !=
                          [
                            'FamilyName',
                            'FontName',
                            'FullName',
                            'Notice',
                            'version',
                            'Copyright',
                          ].indexOf(o) && (t[o] = a[t[o] - 426 + 35]);
                    }),
                    (e.CFF.readSubrs = function (r, t, a) {
                      var n = e._bin,
                        o = [];
                      t = e.CFF.readIndex(r, t, o);
                      var s,
                        i = o.length;
                      (s = i < 1240 ? 107 : i < 33900 ? 1131 : 32768), (a.Bias = s), (a.Subrs = []);
                      for (var h = 0; h < o.length - 1; h++)
                        a.Subrs.push(n.readBytes(r, t + o[h], o[h + 1] - o[h]));
                    }),
                    (e.CFF.tableSE = [
                      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                      0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17,
                      18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36,
                      37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55,
                      56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74,
                      75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93,
                      94, 95, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105,
                      106, 107, 108, 109, 110, 0, 111, 112, 113, 114, 0, 115, 116, 117, 118, 119,
                      120, 121, 122, 0, 123, 0, 124, 125, 126, 127, 128, 129, 130, 131, 0, 132, 133,
                      0, 134, 135, 136, 137, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 138, 0,
                      139, 0, 0, 0, 0, 140, 141, 142, 143, 0, 0, 0, 0, 0, 144, 0, 0, 0, 145, 0, 0,
                      146, 147, 148, 149, 0, 0, 0, 0,
                    ]),
                    (e.CFF.glyphByUnicode = function (r, e) {
                      for (var t = 0; t < r.charset.length; t++) if (r.charset[t] == e) return t;
                      return -1;
                    }),
                    (e.CFF.glyphBySE = function (r, t) {
                      return t < 0 || t > 255 ? -1 : e.CFF.glyphByUnicode(r, e.CFF.tableSE[t]);
                    }),
                    (e.CFF.readEncoding = function (r, t, a) {
                      e._bin;
                      var n = ['.notdef'],
                        o = r[t];
                      if ((t++, 0 != o)) throw 'error: unknown encoding format: ' + o;
                      var s = r[t];
                      t++;
                      for (var i = 0; i < s; i++) n.push(r[t + i]);
                      return n;
                    }),
                    (e.CFF.readCharset = function (r, t, a) {
                      var n = e._bin,
                        o = ['.notdef'],
                        s = r[t];
                      if ((t++, 0 == s))
                        for (var i = 0; i < a; i++) {
                          var h = n.readUshort(r, t);
                          (t += 2), o.push(h);
                        }
                      else {
                        if (1 != s && 2 != s) throw 'error: format: ' + s;
                        for (; o.length < a; ) {
                          (h = n.readUshort(r, t)), (t += 2);
                          var d = 0;
                          for (
                            1 == s ? ((d = r[t]), t++) : ((d = n.readUshort(r, t)), (t += 2)),
                              i = 0;
                            i <= d;
                            i++
                          )
                            o.push(h), h++;
                        }
                      }
                      return o;
                    }),
                    (e.CFF.readIndex = function (r, t, a) {
                      var n = e._bin,
                        o = n.readUshort(r, t) + 1,
                        s = r[(t += 2)];
                      if ((t++, 1 == s)) for (var i = 0; i < o; i++) a.push(r[t + i]);
                      else if (2 == s) for (i = 0; i < o; i++) a.push(n.readUshort(r, t + 2 * i));
                      else if (3 == s)
                        for (i = 0; i < o; i++) a.push(16777215 & n.readUint(r, t + 3 * i - 1));
                      else if (1 != o) throw 'unsupported offset size: ' + s + ', count: ' + o;
                      return (t += o * s) - 1;
                    }),
                    (e.CFF.getCharString = function (r, t, a) {
                      var n = e._bin,
                        o = r[t],
                        s = r[t + 1];
                      r[t + 2], r[t + 3], r[t + 4];
                      var i = 1,
                        h = null,
                        d = null;
                      o <= 20 && ((h = o), (i = 1)),
                        12 == o && ((h = 100 * o + s), (i = 2)),
                        21 <= o && o <= 27 && ((h = o), (i = 1)),
                        28 == o && ((d = n.readShort(r, t + 1)), (i = 3)),
                        29 <= o && o <= 31 && ((h = o), (i = 1)),
                        32 <= o && o <= 246 && ((d = o - 139), (i = 1)),
                        247 <= o && o <= 250 && ((d = 256 * (o - 247) + s + 108), (i = 2)),
                        251 <= o && o <= 254 && ((d = 256 * -(o - 251) - s - 108), (i = 2)),
                        255 == o && ((d = n.readInt(r, t + 1) / 65535), (i = 5)),
                        (a.val = null != d ? d : 'o' + h),
                        (a.size = i);
                    }),
                    (e.CFF.readCharString = function (r, t, a) {
                      for (var n = t + a, o = e._bin, s = []; t < n; ) {
                        var i = r[t],
                          h = r[t + 1];
                        r[t + 2], r[t + 3], r[t + 4];
                        var d = 1,
                          f = null,
                          u = null;
                        i <= 20 && ((f = i), (d = 1)),
                          12 == i && ((f = 100 * i + h), (d = 2)),
                          (19 != i && 20 != i) || ((f = i), (d = 2)),
                          21 <= i && i <= 27 && ((f = i), (d = 1)),
                          28 == i && ((u = o.readShort(r, t + 1)), (d = 3)),
                          29 <= i && i <= 31 && ((f = i), (d = 1)),
                          32 <= i && i <= 246 && ((u = i - 139), (d = 1)),
                          247 <= i && i <= 250 && ((u = 256 * (i - 247) + h + 108), (d = 2)),
                          251 <= i && i <= 254 && ((u = 256 * -(i - 251) - h - 108), (d = 2)),
                          255 == i && ((u = o.readInt(r, t + 1) / 65535), (d = 5)),
                          s.push(null != u ? u : 'o' + f),
                          (t += d);
                      }
                      return s;
                    }),
                    (e.CFF.readDict = function (r, t, a) {
                      for (var n = e._bin, o = {}, s = []; t < a; ) {
                        var i = r[t],
                          h = r[t + 1];
                        r[t + 2], r[t + 3], r[t + 4];
                        var d = 1,
                          f = null,
                          u = null;
                        if (
                          (28 == i && ((u = n.readShort(r, t + 1)), (d = 3)),
                          29 == i && ((u = n.readInt(r, t + 1)), (d = 5)),
                          32 <= i && i <= 246 && ((u = i - 139), (d = 1)),
                          247 <= i && i <= 250 && ((u = 256 * (i - 247) + h + 108), (d = 2)),
                          251 <= i && i <= 254 && ((u = 256 * -(i - 251) - h - 108), (d = 2)),
                          255 == i)
                        )
                          throw ((u = n.readInt(r, t + 1) / 65535), (d = 5), 'unknown number');
                        if (30 == i) {
                          var l = [];
                          for (d = 1; ; ) {
                            var v = r[t + d];
                            d++;
                            var c = v >> 4,
                              p = 15 & v;
                            if ((15 != c && l.push(c), 15 != p && l.push(p), 15 == p)) break;
                          }
                          for (
                            var U = '',
                              g = [
                                0,
                                1,
                                2,
                                3,
                                4,
                                5,
                                6,
                                7,
                                8,
                                9,
                                '.',
                                'e',
                                'e-',
                                'reserved',
                                '-',
                                'endOfNumber',
                              ],
                              S = 0;
                            S < l.length;
                            S++
                          )
                            U += g[l[S]];
                          u = parseFloat(U);
                        }
                        i <= 21 &&
                          ((f = [
                            'version',
                            'Notice',
                            'FullName',
                            'FamilyName',
                            'Weight',
                            'FontBBox',
                            'BlueValues',
                            'OtherBlues',
                            'FamilyBlues',
                            'FamilyOtherBlues',
                            'StdHW',
                            'StdVW',
                            'escape',
                            'UniqueID',
                            'XUID',
                            'charset',
                            'Encoding',
                            'CharStrings',
                            'Private',
                            'Subrs',
                            'defaultWidthX',
                            'nominalWidthX',
                          ][i]),
                          (d = 1),
                          12 == i &&
                            ((f = [
                              'Copyright',
                              'isFixedPitch',
                              'ItalicAngle',
                              'UnderlinePosition',
                              'UnderlineThickness',
                              'PaintType',
                              'CharstringType',
                              'FontMatrix',
                              'StrokeWidth',
                              'BlueScale',
                              'BlueShift',
                              'BlueFuzz',
                              'StemSnapH',
                              'StemSnapV',
                              'ForceBold',
                              0,
                              0,
                              'LanguageGroup',
                              'ExpansionFactor',
                              'initialRandomSeed',
                              'SyntheticBase',
                              'PostScript',
                              'BaseFontName',
                              'BaseFontBlend',
                              0,
                              0,
                              0,
                              0,
                              0,
                              0,
                              'ROS',
                              'CIDFontVersion',
                              'CIDFontRevision',
                              'CIDFontType',
                              'CIDCount',
                              'UIDBase',
                              'FDArray',
                              'FDSelect',
                              'FontName',
                            ][h]),
                            (d = 2))),
                          null != f ? ((o[f] = 1 == s.length ? s[0] : s), (s = [])) : s.push(u),
                          (t += d);
                      }
                      return o;
                    }),
                    (e.cmap = {}),
                    (e.cmap.parse = function (r, t, a) {
                      (r = new Uint8Array(r.buffer, t, a)), (t = 0);
                      var n = e._bin,
                        o = {};
                      n.readUshort(r, t), (t += 2);
                      var s = n.readUshort(r, t);
                      t += 2;
                      var i = [];
                      o.tables = [];
                      for (var h = 0; h < s; h++) {
                        var d = n.readUshort(r, t);
                        t += 2;
                        var f = n.readUshort(r, t);
                        t += 2;
                        var u = n.readUint(r, t);
                        t += 4;
                        var l = 'p' + d + 'e' + f,
                          v = i.indexOf(u);
                        if (-1 == v) {
                          var c;
                          (v = o.tables.length), i.push(u);
                          var p = n.readUshort(r, u);
                          0 == p
                            ? (c = e.cmap.parse0(r, u))
                            : 4 == p
                              ? (c = e.cmap.parse4(r, u))
                              : 6 == p
                                ? (c = e.cmap.parse6(r, u))
                                : 12 == p
                                  ? (c = e.cmap.parse12(r, u))
                                  : console.debug('unknown format: ' + p, d, f, u),
                            o.tables.push(c);
                        }
                        if (null != o[l]) throw 'multiple tables for one platform+encoding';
                        o[l] = v;
                      }
                      return o;
                    }),
                    (e.cmap.parse0 = function (r, t) {
                      var a = e._bin,
                        n = {};
                      (n.format = a.readUshort(r, t)), (t += 2);
                      var o = a.readUshort(r, t);
                      (t += 2), a.readUshort(r, t), (t += 2), (n.map = []);
                      for (var s = 0; s < o - 6; s++) n.map.push(r[t + s]);
                      return n;
                    }),
                    (e.cmap.parse4 = function (r, t) {
                      var a = e._bin,
                        n = t,
                        o = {};
                      (o.format = a.readUshort(r, t)), (t += 2);
                      var s = a.readUshort(r, t);
                      (t += 2), a.readUshort(r, t), (t += 2);
                      var i = a.readUshort(r, t);
                      t += 2;
                      var h = i / 2;
                      (o.searchRange = a.readUshort(r, t)),
                        (t += 2),
                        (o.entrySelector = a.readUshort(r, t)),
                        (t += 2),
                        (o.rangeShift = a.readUshort(r, t)),
                        (t += 2),
                        (o.endCount = a.readUshorts(r, t, h)),
                        (t += 2 * h),
                        (t += 2),
                        (o.startCount = a.readUshorts(r, t, h)),
                        (t += 2 * h),
                        (o.idDelta = []);
                      for (var d = 0; d < h; d++) o.idDelta.push(a.readShort(r, t)), (t += 2);
                      for (
                        o.idRangeOffset = a.readUshorts(r, t, h), t += 2 * h, o.glyphIdArray = [];
                        t < n + s;

                      )
                        o.glyphIdArray.push(a.readUshort(r, t)), (t += 2);
                      return o;
                    }),
                    (e.cmap.parse6 = function (r, t) {
                      var a = e._bin,
                        n = {};
                      (n.format = a.readUshort(r, t)),
                        (t += 2),
                        a.readUshort(r, t),
                        (t += 2),
                        a.readUshort(r, t),
                        (t += 2),
                        (n.firstCode = a.readUshort(r, t)),
                        (t += 2);
                      var o = a.readUshort(r, t);
                      (t += 2), (n.glyphIdArray = []);
                      for (var s = 0; s < o; s++) n.glyphIdArray.push(a.readUshort(r, t)), (t += 2);
                      return n;
                    }),
                    (e.cmap.parse12 = function (r, t) {
                      var a = e._bin,
                        n = {};
                      (n.format = a.readUshort(r, t)),
                        (t += 2),
                        (t += 2),
                        a.readUint(r, t),
                        (t += 4),
                        a.readUint(r, t),
                        (t += 4);
                      var o = a.readUint(r, t);
                      (t += 4), (n.groups = []);
                      for (var s = 0; s < o; s++) {
                        var i = t + 12 * s,
                          h = a.readUint(r, i + 0),
                          d = a.readUint(r, i + 4),
                          f = a.readUint(r, i + 8);
                        n.groups.push([h, d, f]);
                      }
                      return n;
                    }),
                    (e.glyf = {}),
                    (e.glyf.parse = function (r, e, t, a) {
                      for (var n = [], o = 0; o < a.maxp.numGlyphs; o++) n.push(null);
                      return n;
                    }),
                    (e.glyf._parseGlyf = function (r, t) {
                      var a = e._bin,
                        n = r._data,
                        o = e._tabOffset(n, 'glyf', r._offset) + r.loca[t];
                      if (r.loca[t] == r.loca[t + 1]) return null;
                      var s = {};
                      if (
                        ((s.noc = a.readShort(n, o)),
                        (o += 2),
                        (s.xMin = a.readShort(n, o)),
                        (o += 2),
                        (s.yMin = a.readShort(n, o)),
                        (o += 2),
                        (s.xMax = a.readShort(n, o)),
                        (o += 2),
                        (s.yMax = a.readShort(n, o)),
                        (o += 2),
                        s.xMin >= s.xMax || s.yMin >= s.yMax)
                      )
                        return null;
                      if (s.noc > 0) {
                        s.endPts = [];
                        for (var i = 0; i < s.noc; i++) s.endPts.push(a.readUshort(n, o)), (o += 2);
                        var h = a.readUshort(n, o);
                        if (((o += 2), n.length - o < h)) return null;
                        (s.instructions = a.readBytes(n, o, h)), (o += h);
                        var d = s.endPts[s.noc - 1] + 1;
                        for (s.flags = [], i = 0; i < d; i++) {
                          var f = n[o];
                          if ((o++, s.flags.push(f), 8 & f)) {
                            var u = n[o];
                            o++;
                            for (var l = 0; l < u; l++) s.flags.push(f), i++;
                          }
                        }
                        for (s.xs = [], i = 0; i < d; i++) {
                          var v = !!(2 & s.flags[i]),
                            c = !!(16 & s.flags[i]);
                          v
                            ? (s.xs.push(c ? n[o] : -n[o]), o++)
                            : c
                              ? s.xs.push(0)
                              : (s.xs.push(a.readShort(n, o)), (o += 2));
                        }
                        for (s.ys = [], i = 0; i < d; i++)
                          (v = !!(4 & s.flags[i])),
                            (c = !!(32 & s.flags[i])),
                            v
                              ? (s.ys.push(c ? n[o] : -n[o]), o++)
                              : c
                                ? s.ys.push(0)
                                : (s.ys.push(a.readShort(n, o)), (o += 2));
                        var p = 0,
                          U = 0;
                        for (i = 0; i < d; i++)
                          (p += s.xs[i]), (U += s.ys[i]), (s.xs[i] = p), (s.ys[i] = U);
                      } else {
                        var g;
                        s.parts = [];
                        do {
                          (g = a.readUshort(n, o)), (o += 2);
                          var S = { m: { a: 1, b: 0, c: 0, d: 1, tx: 0, ty: 0 }, p1: -1, p2: -1 };
                          if (
                            (s.parts.push(S), (S.glyphIndex = a.readUshort(n, o)), (o += 2), 1 & g)
                          ) {
                            var m = a.readShort(n, o);
                            o += 2;
                            var b = a.readShort(n, o);
                            o += 2;
                          } else (m = a.readInt8(n, o)), o++, (b = a.readInt8(n, o)), o++;
                          2 & g ? ((S.m.tx = m), (S.m.ty = b)) : ((S.p1 = m), (S.p2 = b)),
                            8 & g
                              ? ((S.m.a = S.m.d = a.readF2dot14(n, o)), (o += 2))
                              : 64 & g
                                ? ((S.m.a = a.readF2dot14(n, o)),
                                  (o += 2),
                                  (S.m.d = a.readF2dot14(n, o)),
                                  (o += 2))
                                : 128 & g &&
                                  ((S.m.a = a.readF2dot14(n, o)),
                                  (o += 2),
                                  (S.m.b = a.readF2dot14(n, o)),
                                  (o += 2),
                                  (S.m.c = a.readF2dot14(n, o)),
                                  (o += 2),
                                  (S.m.d = a.readF2dot14(n, o)),
                                  (o += 2));
                        } while (32 & g);
                        if (256 & g) {
                          var y = a.readUshort(n, o);
                          for (o += 2, s.instr = [], i = 0; i < y; i++) s.instr.push(n[o]), o++;
                        }
                      }
                      return s;
                    }),
                    (e.GDEF = {}),
                    (e.GDEF.parse = function (r, t, a, n) {
                      var o = t;
                      t += 4;
                      var s = e._bin.readUshort(r, t);
                      return { glyphClassDef: 0 === s ? null : e._lctf.readClassDef(r, o + s) };
                    }),
                    (e.GPOS = {}),
                    (e.GPOS.parse = function (r, t, a, n) {
                      return e._lctf.parse(r, t, a, n, e.GPOS.subt);
                    }),
                    (e.GPOS.subt = function (r, t, a, n) {
                      var o = e._bin,
                        s = a,
                        i = {};
                      if (
                        ((i.fmt = o.readUshort(r, a)),
                        (a += 2),
                        1 == t || 2 == t || 3 == t || 7 == t || (8 == t && i.fmt <= 2))
                      ) {
                        var h = o.readUshort(r, a);
                        (a += 2), (i.coverage = e._lctf.readCoverage(r, h + s));
                      }
                      if (1 == t && 1 == i.fmt) {
                        var d = o.readUshort(r, a);
                        (a += 2), 0 != d && (i.pos = e.GPOS.readValueRecord(r, a, d));
                      } else if (2 == t && i.fmt >= 1 && i.fmt <= 2) {
                        (d = o.readUshort(r, a)), (a += 2);
                        var f = o.readUshort(r, a);
                        a += 2;
                        var u = e._lctf.numOfOnes(d),
                          l = e._lctf.numOfOnes(f);
                        if (1 == i.fmt) {
                          i.pairsets = [];
                          var v = o.readUshort(r, a);
                          a += 2;
                          for (var c = 0; c < v; c++) {
                            var p = s + o.readUshort(r, a);
                            a += 2;
                            var U = o.readUshort(r, p);
                            p += 2;
                            for (var g = [], S = 0; S < U; S++) {
                              var m = o.readUshort(r, p);
                              (p += 2),
                                0 != d && ((P = e.GPOS.readValueRecord(r, p, d)), (p += 2 * u)),
                                0 != f && ((x = e.GPOS.readValueRecord(r, p, f)), (p += 2 * l)),
                                g.push({ gid2: m, val1: P, val2: x });
                            }
                            i.pairsets.push(g);
                          }
                        }
                        if (2 == i.fmt) {
                          var b = o.readUshort(r, a);
                          a += 2;
                          var y = o.readUshort(r, a);
                          a += 2;
                          var F = o.readUshort(r, a);
                          a += 2;
                          var C = o.readUshort(r, a);
                          for (
                            a += 2,
                              i.classDef1 = e._lctf.readClassDef(r, s + b),
                              i.classDef2 = e._lctf.readClassDef(r, s + y),
                              i.matrix = [],
                              c = 0;
                            c < F;
                            c++
                          ) {
                            var _ = [];
                            for (S = 0; S < C; S++) {
                              var P = null,
                                x = null;
                              0 != d && ((P = e.GPOS.readValueRecord(r, a, d)), (a += 2 * u)),
                                0 != f && ((x = e.GPOS.readValueRecord(r, a, f)), (a += 2 * l)),
                                _.push({ val1: P, val2: x });
                            }
                            i.matrix.push(_);
                          }
                        }
                      } else if (4 == t && 1 == i.fmt)
                        (i.markCoverage = e._lctf.readCoverage(r, o.readUshort(r, a) + s)),
                          (i.baseCoverage = e._lctf.readCoverage(r, o.readUshort(r, a + 2) + s)),
                          (i.markClassCount = o.readUshort(r, a + 4)),
                          (i.markArray = e.GPOS.readMarkArray(r, o.readUshort(r, a + 6) + s)),
                          (i.baseArray = e.GPOS.readBaseArray(
                            r,
                            o.readUshort(r, a + 8) + s,
                            i.markClassCount,
                          ));
                      else if (6 == t && 1 == i.fmt)
                        (i.mark1Coverage = e._lctf.readCoverage(r, o.readUshort(r, a) + s)),
                          (i.mark2Coverage = e._lctf.readCoverage(r, o.readUshort(r, a + 2) + s)),
                          (i.markClassCount = o.readUshort(r, a + 4)),
                          (i.mark1Array = e.GPOS.readMarkArray(r, o.readUshort(r, a + 6) + s)),
                          (i.mark2Array = e.GPOS.readBaseArray(
                            r,
                            o.readUshort(r, a + 8) + s,
                            i.markClassCount,
                          ));
                      else {
                        if (9 == t && 1 == i.fmt) {
                          var I = o.readUshort(r, a);
                          a += 2;
                          var w = o.readUint(r, a);
                          if (((a += 4), 9 == n.ltype)) n.ltype = I;
                          else if (n.ltype != I) throw 'invalid extension substitution';
                          return e.GPOS.subt(r, n.ltype, s + w);
                        }
                        console.debug('unsupported GPOS table LookupType', t, 'format', i.fmt);
                      }
                      return i;
                    }),
                    (e.GPOS.readValueRecord = function (r, t, a) {
                      var n = e._bin,
                        o = [];
                      return (
                        o.push(1 & a ? n.readShort(r, t) : 0),
                        (t += 1 & a ? 2 : 0),
                        o.push(2 & a ? n.readShort(r, t) : 0),
                        (t += 2 & a ? 2 : 0),
                        o.push(4 & a ? n.readShort(r, t) : 0),
                        (t += 4 & a ? 2 : 0),
                        o.push(8 & a ? n.readShort(r, t) : 0),
                        (t += 8 & a ? 2 : 0),
                        o
                      );
                    }),
                    (e.GPOS.readBaseArray = function (r, t, a) {
                      var n = e._bin,
                        o = [],
                        s = t,
                        i = n.readUshort(r, t);
                      t += 2;
                      for (var h = 0; h < i; h++) {
                        for (var d = [], f = 0; f < a; f++)
                          d.push(e.GPOS.readAnchorRecord(r, s + n.readUshort(r, t))), (t += 2);
                        o.push(d);
                      }
                      return o;
                    }),
                    (e.GPOS.readMarkArray = function (r, t) {
                      var a = e._bin,
                        n = [],
                        o = t,
                        s = a.readUshort(r, t);
                      t += 2;
                      for (var i = 0; i < s; i++) {
                        var h = e.GPOS.readAnchorRecord(r, a.readUshort(r, t + 2) + o);
                        (h.markClass = a.readUshort(r, t)), n.push(h), (t += 4);
                      }
                      return n;
                    }),
                    (e.GPOS.readAnchorRecord = function (r, t) {
                      var a = e._bin,
                        n = {};
                      return (
                        (n.fmt = a.readUshort(r, t)),
                        (n.x = a.readShort(r, t + 2)),
                        (n.y = a.readShort(r, t + 4)),
                        n
                      );
                    }),
                    (e.GSUB = {}),
                    (e.GSUB.parse = function (r, t, a, n) {
                      return e._lctf.parse(r, t, a, n, e.GSUB.subt);
                    }),
                    (e.GSUB.subt = function (r, t, a, n) {
                      var o = e._bin,
                        s = a,
                        i = {};
                      if (
                        ((i.fmt = o.readUshort(r, a)),
                        (a += 2),
                        1 != t && 2 != t && 4 != t && 5 != t && 6 != t)
                      )
                        return null;
                      if (
                        1 == t ||
                        2 == t ||
                        4 == t ||
                        (5 == t && i.fmt <= 2) ||
                        (6 == t && i.fmt <= 2)
                      ) {
                        var h = o.readUshort(r, a);
                        (a += 2), (i.coverage = e._lctf.readCoverage(r, s + h));
                      }
                      if (1 == t && i.fmt >= 1 && i.fmt <= 2) {
                        if (1 == i.fmt) (i.delta = o.readShort(r, a)), (a += 2);
                        else if (2 == i.fmt) {
                          var d = o.readUshort(r, a);
                          (a += 2), (i.newg = o.readUshorts(r, a, d)), (a += 2 * i.newg.length);
                        }
                      } else if (2 == t && 1 == i.fmt) {
                        (d = o.readUshort(r, a)), (a += 2), (i.seqs = []);
                        for (var f = 0; f < d; f++) {
                          var u = o.readUshort(r, a) + s;
                          a += 2;
                          var l = o.readUshort(r, u);
                          i.seqs.push(o.readUshorts(r, u + 2, l));
                        }
                      } else if (4 == t)
                        for (i.vals = [], d = o.readUshort(r, a), a += 2, f = 0; f < d; f++) {
                          var v = o.readUshort(r, a);
                          (a += 2), i.vals.push(e.GSUB.readLigatureSet(r, s + v));
                        }
                      else if (5 == t && 2 == i.fmt) {
                        if (2 == i.fmt) {
                          var c = o.readUshort(r, a);
                          (a += 2), (i.cDef = e._lctf.readClassDef(r, s + c)), (i.scset = []);
                          var p = o.readUshort(r, a);
                          for (a += 2, f = 0; f < p; f++) {
                            var U = o.readUshort(r, a);
                            (a += 2),
                              i.scset.push(0 == U ? null : e.GSUB.readSubClassSet(r, s + U));
                          }
                        }
                      } else if (6 == t && 3 == i.fmt) {
                        if (3 == i.fmt) {
                          for (f = 0; f < 3; f++) {
                            (d = o.readUshort(r, a)), (a += 2);
                            for (var g = [], S = 0; S < d; S++)
                              g.push(e._lctf.readCoverage(r, s + o.readUshort(r, a + 2 * S)));
                            (a += 2 * d),
                              0 == f && (i.backCvg = g),
                              1 == f && (i.inptCvg = g),
                              2 == f && (i.ahedCvg = g);
                          }
                          (d = o.readUshort(r, a)),
                            (a += 2),
                            (i.lookupRec = e.GSUB.readSubstLookupRecords(r, a, d));
                        }
                      } else {
                        if (7 == t && 1 == i.fmt) {
                          var m = o.readUshort(r, a);
                          a += 2;
                          var b = o.readUint(r, a);
                          if (((a += 4), 9 == n.ltype)) n.ltype = m;
                          else if (n.ltype != m) throw 'invalid extension substitution';
                          return e.GSUB.subt(r, n.ltype, s + b);
                        }
                        console.debug('unsupported GSUB table LookupType', t, 'format', i.fmt);
                      }
                      return i;
                    }),
                    (e.GSUB.readSubClassSet = function (r, t) {
                      var a = e._bin.readUshort,
                        n = t,
                        o = [],
                        s = a(r, t);
                      t += 2;
                      for (var i = 0; i < s; i++) {
                        var h = a(r, t);
                        (t += 2), o.push(e.GSUB.readSubClassRule(r, n + h));
                      }
                      return o;
                    }),
                    (e.GSUB.readSubClassRule = function (r, t) {
                      var a = e._bin.readUshort,
                        n = {},
                        o = a(r, t),
                        s = a(r, (t += 2));
                      (t += 2), (n.input = []);
                      for (var i = 0; i < o - 1; i++) n.input.push(a(r, t)), (t += 2);
                      return (n.substLookupRecords = e.GSUB.readSubstLookupRecords(r, t, s)), n;
                    }),
                    (e.GSUB.readSubstLookupRecords = function (r, t, a) {
                      for (var n = e._bin.readUshort, o = [], s = 0; s < a; s++)
                        o.push(n(r, t), n(r, t + 2)), (t += 4);
                      return o;
                    }),
                    (e.GSUB.readChainSubClassSet = function (r, t) {
                      var a = e._bin,
                        n = t,
                        o = [],
                        s = a.readUshort(r, t);
                      t += 2;
                      for (var i = 0; i < s; i++) {
                        var h = a.readUshort(r, t);
                        (t += 2), o.push(e.GSUB.readChainSubClassRule(r, n + h));
                      }
                      return o;
                    }),
                    (e.GSUB.readChainSubClassRule = function (r, t) {
                      for (
                        var a = e._bin, n = {}, o = ['backtrack', 'input', 'lookahead'], s = 0;
                        s < o.length;
                        s++
                      ) {
                        var i = a.readUshort(r, t);
                        (t += 2),
                          1 == s && i--,
                          (n[o[s]] = a.readUshorts(r, t, i)),
                          (t += 2 * n[o[s]].length);
                      }
                      return (
                        (i = a.readUshort(r, t)),
                        (t += 2),
                        (n.subst = a.readUshorts(r, t, 2 * i)),
                        (t += 2 * n.subst.length),
                        n
                      );
                    }),
                    (e.GSUB.readLigatureSet = function (r, t) {
                      var a = e._bin,
                        n = t,
                        o = [],
                        s = a.readUshort(r, t);
                      t += 2;
                      for (var i = 0; i < s; i++) {
                        var h = a.readUshort(r, t);
                        (t += 2), o.push(e.GSUB.readLigature(r, n + h));
                      }
                      return o;
                    }),
                    (e.GSUB.readLigature = function (r, t) {
                      var a = e._bin,
                        n = { chain: [] };
                      (n.nglyph = a.readUshort(r, t)), (t += 2);
                      var o = a.readUshort(r, t);
                      t += 2;
                      for (var s = 0; s < o - 1; s++) n.chain.push(a.readUshort(r, t)), (t += 2);
                      return n;
                    }),
                    (e.head = {}),
                    (e.head.parse = function (r, t, a) {
                      var n = e._bin,
                        o = {};
                      return (
                        n.readFixed(r, t),
                        (t += 4),
                        (o.fontRevision = n.readFixed(r, t)),
                        (t += 4),
                        n.readUint(r, t),
                        (t += 4),
                        n.readUint(r, t),
                        (t += 4),
                        (o.flags = n.readUshort(r, t)),
                        (t += 2),
                        (o.unitsPerEm = n.readUshort(r, t)),
                        (t += 2),
                        (o.created = n.readUint64(r, t)),
                        (t += 8),
                        (o.modified = n.readUint64(r, t)),
                        (t += 8),
                        (o.xMin = n.readShort(r, t)),
                        (t += 2),
                        (o.yMin = n.readShort(r, t)),
                        (t += 2),
                        (o.xMax = n.readShort(r, t)),
                        (t += 2),
                        (o.yMax = n.readShort(r, t)),
                        (t += 2),
                        (o.macStyle = n.readUshort(r, t)),
                        (t += 2),
                        (o.lowestRecPPEM = n.readUshort(r, t)),
                        (t += 2),
                        (o.fontDirectionHint = n.readShort(r, t)),
                        (t += 2),
                        (o.indexToLocFormat = n.readShort(r, t)),
                        (t += 2),
                        (o.glyphDataFormat = n.readShort(r, t)),
                        (t += 2),
                        o
                      );
                    }),
                    (e.hhea = {}),
                    (e.hhea.parse = function (r, t, a) {
                      var n = e._bin,
                        o = {};
                      return (
                        n.readFixed(r, t),
                        (t += 4),
                        (o.ascender = n.readShort(r, t)),
                        (t += 2),
                        (o.descender = n.readShort(r, t)),
                        (t += 2),
                        (o.lineGap = n.readShort(r, t)),
                        (t += 2),
                        (o.advanceWidthMax = n.readUshort(r, t)),
                        (t += 2),
                        (o.minLeftSideBearing = n.readShort(r, t)),
                        (t += 2),
                        (o.minRightSideBearing = n.readShort(r, t)),
                        (t += 2),
                        (o.xMaxExtent = n.readShort(r, t)),
                        (t += 2),
                        (o.caretSlopeRise = n.readShort(r, t)),
                        (t += 2),
                        (o.caretSlopeRun = n.readShort(r, t)),
                        (t += 2),
                        (o.caretOffset = n.readShort(r, t)),
                        (t += 2),
                        (t += 8),
                        (o.metricDataFormat = n.readShort(r, t)),
                        (t += 2),
                        (o.numberOfHMetrics = n.readUshort(r, t)),
                        (t += 2),
                        o
                      );
                    }),
                    (e.hmtx = {}),
                    (e.hmtx.parse = function (r, t, a, n) {
                      for (
                        var o = e._bin, s = { aWidth: [], lsBearing: [] }, i = 0, h = 0, d = 0;
                        d < n.maxp.numGlyphs;
                        d++
                      )
                        d < n.hhea.numberOfHMetrics &&
                          ((i = o.readUshort(r, t)), (t += 2), (h = o.readShort(r, t)), (t += 2)),
                          s.aWidth.push(i),
                          s.lsBearing.push(h);
                      return s;
                    }),
                    (e.kern = {}),
                    (e.kern.parse = function (r, t, a, n) {
                      var o = e._bin,
                        s = o.readUshort(r, t);
                      if (((t += 2), 1 == s)) return e.kern.parseV1(r, t - 2, a, n);
                      var i = o.readUshort(r, t);
                      t += 2;
                      for (var h = { glyph1: [], rval: [] }, d = 0; d < i; d++) {
                        (t += 2), (a = o.readUshort(r, t)), (t += 2);
                        var f = o.readUshort(r, t);
                        t += 2;
                        var u = f >>> 8;
                        if (0 != (u &= 15)) throw 'unknown kern table format: ' + u;
                        t = e.kern.readFormat0(r, t, h);
                      }
                      return h;
                    }),
                    (e.kern.parseV1 = function (r, t, a, n) {
                      var o = e._bin;
                      o.readFixed(r, t), (t += 4);
                      var s = o.readUint(r, t);
                      t += 4;
                      for (var i = { glyph1: [], rval: [] }, h = 0; h < s; h++) {
                        o.readUint(r, t), (t += 4);
                        var d = o.readUshort(r, t);
                        (t += 2), o.readUshort(r, t), (t += 2);
                        var f = d >>> 8;
                        if (0 != (f &= 15)) throw 'unknown kern table format: ' + f;
                        t = e.kern.readFormat0(r, t, i);
                      }
                      return i;
                    }),
                    (e.kern.readFormat0 = function (r, t, a) {
                      var n = e._bin,
                        o = -1,
                        s = n.readUshort(r, t);
                      (t += 2),
                        n.readUshort(r, t),
                        (t += 2),
                        n.readUshort(r, t),
                        (t += 2),
                        n.readUshort(r, t),
                        (t += 2);
                      for (var i = 0; i < s; i++) {
                        var h = n.readUshort(r, t);
                        t += 2;
                        var d = n.readUshort(r, t);
                        t += 2;
                        var f = n.readShort(r, t);
                        (t += 2),
                          h != o && (a.glyph1.push(h), a.rval.push({ glyph2: [], vals: [] }));
                        var u = a.rval[a.rval.length - 1];
                        u.glyph2.push(d), u.vals.push(f), (o = h);
                      }
                      return t;
                    }),
                    (e.loca = {}),
                    (e.loca.parse = function (r, t, a, n) {
                      var o = e._bin,
                        s = [],
                        i = n.head.indexToLocFormat,
                        h = n.maxp.numGlyphs + 1;
                      if (0 == i)
                        for (var d = 0; d < h; d++) s.push(o.readUshort(r, t + (d << 1)) << 1);
                      if (1 == i) for (d = 0; d < h; d++) s.push(o.readUint(r, t + (d << 2)));
                      return s;
                    }),
                    (e.maxp = {}),
                    (e.maxp.parse = function (r, t, a) {
                      var n = e._bin,
                        o = {},
                        s = n.readUint(r, t);
                      return (
                        (t += 4),
                        (o.numGlyphs = n.readUshort(r, t)),
                        (t += 2),
                        65536 == s &&
                          ((o.maxPoints = n.readUshort(r, t)),
                          (t += 2),
                          (o.maxContours = n.readUshort(r, t)),
                          (t += 2),
                          (o.maxCompositePoints = n.readUshort(r, t)),
                          (t += 2),
                          (o.maxCompositeContours = n.readUshort(r, t)),
                          (t += 2),
                          (o.maxZones = n.readUshort(r, t)),
                          (t += 2),
                          (o.maxTwilightPoints = n.readUshort(r, t)),
                          (t += 2),
                          (o.maxStorage = n.readUshort(r, t)),
                          (t += 2),
                          (o.maxFunctionDefs = n.readUshort(r, t)),
                          (t += 2),
                          (o.maxInstructionDefs = n.readUshort(r, t)),
                          (t += 2),
                          (o.maxStackElements = n.readUshort(r, t)),
                          (t += 2),
                          (o.maxSizeOfInstructions = n.readUshort(r, t)),
                          (t += 2),
                          (o.maxComponentElements = n.readUshort(r, t)),
                          (t += 2),
                          (o.maxComponentDepth = n.readUshort(r, t)),
                          (t += 2)),
                        o
                      );
                    }),
                    (e.name = {}),
                    (e.name.parse = function (r, t, a) {
                      var n = e._bin,
                        o = {};
                      n.readUshort(r, t), (t += 2);
                      var s = n.readUshort(r, t);
                      (t += 2), n.readUshort(r, t);
                      for (
                        var i,
                          h = [
                            'copyright',
                            'fontFamily',
                            'fontSubfamily',
                            'ID',
                            'fullName',
                            'version',
                            'postScriptName',
                            'trademark',
                            'manufacturer',
                            'designer',
                            'description',
                            'urlVendor',
                            'urlDesigner',
                            'licence',
                            'licenceURL',
                            '---',
                            'typoFamilyName',
                            'typoSubfamilyName',
                            'compatibleFull',
                            'sampleText',
                            'postScriptCID',
                            'wwsFamilyName',
                            'wwsSubfamilyName',
                            'lightPalette',
                            'darkPalette',
                          ],
                          d = (t += 2),
                          f = 0;
                        f < s;
                        f++
                      ) {
                        var u = n.readUshort(r, t);
                        t += 2;
                        var l = n.readUshort(r, t);
                        t += 2;
                        var v = n.readUshort(r, t);
                        t += 2;
                        var c = n.readUshort(r, t);
                        t += 2;
                        var p = n.readUshort(r, t);
                        t += 2;
                        var U = n.readUshort(r, t);
                        t += 2;
                        var g,
                          S = h[c],
                          m = d + 12 * s + U;
                        if (0 == u) g = n.readUnicode(r, m, p / 2);
                        else if (3 == u && 0 == l) g = n.readUnicode(r, m, p / 2);
                        else if (0 == l) g = n.readASCII(r, m, p);
                        else if (1 == l) g = n.readUnicode(r, m, p / 2);
                        else if (3 == l) g = n.readUnicode(r, m, p / 2);
                        else {
                          if (1 != u) throw 'unknown encoding ' + l + ', platformID: ' + u;
                          (g = n.readASCII(r, m, p)),
                            console.debug('reading unknown MAC encoding ' + l + ' as ASCII');
                        }
                        var b = 'p' + u + ',' + v.toString(16);
                        null == o[b] && (o[b] = {}),
                          (o[b][void 0 !== S ? S : c] = g),
                          (o[b]._lang = v);
                      }
                      for (var y in o)
                        if (null != o[y].postScriptName && 1033 == o[y]._lang) return o[y];
                      for (var y in o)
                        if (null != o[y].postScriptName && 0 == o[y]._lang) return o[y];
                      for (var y in o)
                        if (null != o[y].postScriptName && 3084 == o[y]._lang) return o[y];
                      for (var y in o) if (null != o[y].postScriptName) return o[y];
                      for (var y in o) {
                        i = y;
                        break;
                      }
                      return (
                        console.debug('returning name table with languageID ' + o[i]._lang), o[i]
                      );
                    }),
                    (e['OS/2'] = {}),
                    (e['OS/2'].parse = function (r, t, a) {
                      var n = e._bin.readUshort(r, t);
                      t += 2;
                      var o = {};
                      if (0 == n) e['OS/2'].version0(r, t, o);
                      else if (1 == n) e['OS/2'].version1(r, t, o);
                      else if (2 == n || 3 == n || 4 == n) e['OS/2'].version2(r, t, o);
                      else {
                        if (5 != n) throw 'unknown OS/2 table version: ' + n;
                        e['OS/2'].version5(r, t, o);
                      }
                      return o;
                    }),
                    (e['OS/2'].version0 = function (r, t, a) {
                      var n = e._bin;
                      return (
                        (a.xAvgCharWidth = n.readShort(r, t)),
                        (t += 2),
                        (a.usWeightClass = n.readUshort(r, t)),
                        (t += 2),
                        (a.usWidthClass = n.readUshort(r, t)),
                        (t += 2),
                        (a.fsType = n.readUshort(r, t)),
                        (t += 2),
                        (a.ySubscriptXSize = n.readShort(r, t)),
                        (t += 2),
                        (a.ySubscriptYSize = n.readShort(r, t)),
                        (t += 2),
                        (a.ySubscriptXOffset = n.readShort(r, t)),
                        (t += 2),
                        (a.ySubscriptYOffset = n.readShort(r, t)),
                        (t += 2),
                        (a.ySuperscriptXSize = n.readShort(r, t)),
                        (t += 2),
                        (a.ySuperscriptYSize = n.readShort(r, t)),
                        (t += 2),
                        (a.ySuperscriptXOffset = n.readShort(r, t)),
                        (t += 2),
                        (a.ySuperscriptYOffset = n.readShort(r, t)),
                        (t += 2),
                        (a.yStrikeoutSize = n.readShort(r, t)),
                        (t += 2),
                        (a.yStrikeoutPosition = n.readShort(r, t)),
                        (t += 2),
                        (a.sFamilyClass = n.readShort(r, t)),
                        (t += 2),
                        (a.panose = n.readBytes(r, t, 10)),
                        (t += 10),
                        (a.ulUnicodeRange1 = n.readUint(r, t)),
                        (t += 4),
                        (a.ulUnicodeRange2 = n.readUint(r, t)),
                        (t += 4),
                        (a.ulUnicodeRange3 = n.readUint(r, t)),
                        (t += 4),
                        (a.ulUnicodeRange4 = n.readUint(r, t)),
                        (t += 4),
                        (a.achVendID = [
                          n.readInt8(r, t),
                          n.readInt8(r, t + 1),
                          n.readInt8(r, t + 2),
                          n.readInt8(r, t + 3),
                        ]),
                        (t += 4),
                        (a.fsSelection = n.readUshort(r, t)),
                        (t += 2),
                        (a.usFirstCharIndex = n.readUshort(r, t)),
                        (t += 2),
                        (a.usLastCharIndex = n.readUshort(r, t)),
                        (t += 2),
                        (a.sTypoAscender = n.readShort(r, t)),
                        (t += 2),
                        (a.sTypoDescender = n.readShort(r, t)),
                        (t += 2),
                        (a.sTypoLineGap = n.readShort(r, t)),
                        (t += 2),
                        (a.usWinAscent = n.readUshort(r, t)),
                        (t += 2),
                        (a.usWinDescent = n.readUshort(r, t)),
                        t + 2
                      );
                    }),
                    (e['OS/2'].version1 = function (r, t, a) {
                      var n = e._bin;
                      return (
                        (t = e['OS/2'].version0(r, t, a)),
                        (a.ulCodePageRange1 = n.readUint(r, t)),
                        (t += 4),
                        (a.ulCodePageRange2 = n.readUint(r, t)),
                        t + 4
                      );
                    }),
                    (e['OS/2'].version2 = function (r, t, a) {
                      var n = e._bin;
                      return (
                        (t = e['OS/2'].version1(r, t, a)),
                        (a.sxHeight = n.readShort(r, t)),
                        (t += 2),
                        (a.sCapHeight = n.readShort(r, t)),
                        (t += 2),
                        (a.usDefault = n.readUshort(r, t)),
                        (t += 2),
                        (a.usBreak = n.readUshort(r, t)),
                        (t += 2),
                        (a.usMaxContext = n.readUshort(r, t)),
                        t + 2
                      );
                    }),
                    (e['OS/2'].version5 = function (r, t, a) {
                      var n = e._bin;
                      return (
                        (t = e['OS/2'].version2(r, t, a)),
                        (a.usLowerOpticalPointSize = n.readUshort(r, t)),
                        (t += 2),
                        (a.usUpperOpticalPointSize = n.readUshort(r, t)),
                        t + 2
                      );
                    }),
                    (e.post = {}),
                    (e.post.parse = function (r, t, a) {
                      var n = e._bin,
                        o = {};
                      return (
                        (o.version = n.readFixed(r, t)),
                        (t += 4),
                        (o.italicAngle = n.readFixed(r, t)),
                        (t += 4),
                        (o.underlinePosition = n.readShort(r, t)),
                        (t += 2),
                        (o.underlineThickness = n.readShort(r, t)),
                        (t += 2),
                        o
                      );
                    }),
                    null == e && (e = {}),
                    null == e.U && (e.U = {}),
                    (e.U.codeToGlyph = function (r, e) {
                      var t = r.cmap,
                        a = -1;
                      if (
                        (null != t.p0e4
                          ? (a = t.p0e4)
                          : null != t.p3e1
                            ? (a = t.p3e1)
                            : null != t.p1e0
                              ? (a = t.p1e0)
                              : null != t.p0e3 && (a = t.p0e3),
                        -1 == a)
                      )
                        throw 'no familiar platform and encoding!';
                      var n = t.tables[a];
                      if (0 == n.format) return e >= n.map.length ? 0 : n.map[e];
                      if (4 == n.format) {
                        for (var o = -1, s = 0; s < n.endCount.length; s++)
                          if (e <= n.endCount[s]) {
                            o = s;
                            break;
                          }
                        return -1 == o || n.startCount[o] > e
                          ? 0
                          : 65535 &
                              (0 != n.idRangeOffset[o]
                                ? n.glyphIdArray[
                                    e -
                                      n.startCount[o] +
                                      (n.idRangeOffset[o] >> 1) -
                                      (n.idRangeOffset.length - o)
                                  ]
                                : e + n.idDelta[o]);
                      }
                      if (12 == n.format) {
                        if (e > n.groups[n.groups.length - 1][1]) return 0;
                        for (s = 0; s < n.groups.length; s++) {
                          var i = n.groups[s];
                          if (i[0] <= e && e <= i[1]) return i[2] + (e - i[0]);
                        }
                        return 0;
                      }
                      throw 'unknown cmap table format ' + n.format;
                    }),
                    (e.U.glyphToPath = function (r, t) {
                      var a = { cmds: [], crds: [] };
                      if (r.SVG && r.SVG.entries[t]) {
                        var n = r.SVG.entries[t];
                        return null == n
                          ? a
                          : ('string' == typeof n &&
                              ((n = e.SVG.toPath(n)), (r.SVG.entries[t] = n)),
                            n);
                      }
                      if (r.CFF) {
                        var o = {
                            x: 0,
                            y: 0,
                            stack: [],
                            nStems: 0,
                            haveWidth: !1,
                            width: r.CFF.Private ? r.CFF.Private.defaultWidthX : 0,
                            open: !1,
                          },
                          s = r.CFF,
                          i = r.CFF.Private;
                        if (s.ROS) {
                          for (var h = 0; s.FDSelect[h + 2] <= t; ) h += 2;
                          i = s.FDArray[s.FDSelect[h + 1]].Private;
                        }
                        e.U._drawCFF(r.CFF.CharStrings[t], o, s, i, a);
                      } else r.glyf && e.U._drawGlyf(t, r, a);
                      return a;
                    }),
                    (e.U._drawGlyf = function (r, t, a) {
                      var n = t.glyf[r];
                      null == n && (n = t.glyf[r] = e.glyf._parseGlyf(t, r)),
                        null != n &&
                          (n.noc > -1 ? e.U._simpleGlyph(n, a) : e.U._compoGlyph(n, t, a));
                    }),
                    (e.U._simpleGlyph = function (r, t) {
                      for (var a = 0; a < r.noc; a++) {
                        for (
                          var n = 0 == a ? 0 : r.endPts[a - 1] + 1, o = r.endPts[a], s = n;
                          s <= o;
                          s++
                        ) {
                          var i = s == n ? o : s - 1,
                            h = s == o ? n : s + 1,
                            d = 1 & r.flags[s],
                            f = 1 & r.flags[i],
                            u = 1 & r.flags[h],
                            l = r.xs[s],
                            v = r.ys[s];
                          if (s == n)
                            if (d) {
                              if (!f) {
                                e.U.P.moveTo(t, l, v);
                                continue;
                              }
                              e.U.P.moveTo(t, r.xs[i], r.ys[i]);
                            } else
                              f
                                ? e.U.P.moveTo(t, r.xs[i], r.ys[i])
                                : e.U.P.moveTo(t, (r.xs[i] + l) / 2, (r.ys[i] + v) / 2);
                          d
                            ? f && e.U.P.lineTo(t, l, v)
                            : u
                              ? e.U.P.qcurveTo(t, l, v, r.xs[h], r.ys[h])
                              : e.U.P.qcurveTo(t, l, v, (l + r.xs[h]) / 2, (v + r.ys[h]) / 2);
                        }
                        e.U.P.closePath(t);
                      }
                    }),
                    (e.U._compoGlyph = function (r, t, a) {
                      for (var n = 0; n < r.parts.length; n++) {
                        var o = { cmds: [], crds: [] },
                          s = r.parts[n];
                        e.U._drawGlyf(s.glyphIndex, t, o);
                        for (var i = s.m, h = 0; h < o.crds.length; h += 2) {
                          var d = o.crds[h],
                            f = o.crds[h + 1];
                          a.crds.push(d * i.a + f * i.b + i.tx),
                            a.crds.push(d * i.c + f * i.d + i.ty);
                        }
                        for (h = 0; h < o.cmds.length; h++) a.cmds.push(o.cmds[h]);
                      }
                    }),
                    (e.U._getGlyphClass = function (r, t) {
                      var a = e._lctf.getInterval(t, r);
                      return -1 == a ? 0 : t[a + 2];
                    }),
                    (e.U._applySubs = function (r, t, a, n) {
                      for (var o = r.length - t - 1, s = 0; s < a.tabs.length; s++)
                        if (null != a.tabs[s]) {
                          var i,
                            h = a.tabs[s];
                          if (!h.coverage || -1 != (i = e._lctf.coverageIndex(h.coverage, r[t])))
                            if (1 == a.ltype)
                              r[t], 1 == h.fmt ? (r[t] = r[t] + h.delta) : (r[t] = h.newg[i]);
                            else if (4 == a.ltype)
                              for (var d = h.vals[i], f = 0; f < d.length; f++) {
                                var u = d[f],
                                  l = u.chain.length;
                                if (!(l > o)) {
                                  for (var v = !0, c = 0, p = 0; p < l; p++) {
                                    for (; -1 == r[t + c + (1 + p)]; ) c++;
                                    u.chain[p] != r[t + c + (1 + p)] && (v = !1);
                                  }
                                  if (v) {
                                    for (r[t] = u.nglyph, p = 0; p < l + c; p++) r[t + p + 1] = -1;
                                    break;
                                  }
                                }
                              }
                            else if (5 == a.ltype && 2 == h.fmt)
                              for (
                                var U = e._lctf.getInterval(h.cDef, r[t]),
                                  g = h.cDef[U + 2],
                                  S = h.scset[g],
                                  m = 0;
                                m < S.length;
                                m++
                              ) {
                                var b = S[m],
                                  y = b.input;
                                if (!(y.length > o)) {
                                  for (v = !0, p = 0; p < y.length; p++) {
                                    var F = e._lctf.getInterval(h.cDef, r[t + 1 + p]);
                                    if (-1 == U && h.cDef[F + 2] != y[p]) {
                                      v = !1;
                                      break;
                                    }
                                  }
                                  if (v) {
                                    var C = b.substLookupRecords;
                                    for (f = 0; f < C.length; f += 2) C[f], C[f + 1];
                                  }
                                }
                              }
                            else if (6 == a.ltype && 3 == h.fmt) {
                              if (!e.U._glsCovered(r, h.backCvg, t - h.backCvg.length)) continue;
                              if (!e.U._glsCovered(r, h.inptCvg, t)) continue;
                              if (!e.U._glsCovered(r, h.ahedCvg, t + h.inptCvg.length)) continue;
                              var _ = h.lookupRec;
                              for (m = 0; m < _.length; m += 2) {
                                U = _[m];
                                var P = n[_[m + 1]];
                                e.U._applySubs(r, t + U, P, n);
                              }
                            }
                        }
                    }),
                    (e.U._glsCovered = function (r, t, a) {
                      for (var n = 0; n < t.length; n++)
                        if (-1 == e._lctf.coverageIndex(t[n], r[a + n])) return !1;
                      return !0;
                    }),
                    (e.U.glyphsToPath = function (r, t, a) {
                      for (var n = { cmds: [], crds: [] }, o = 0, s = 0; s < t.length; s++) {
                        var i = t[s];
                        if (-1 != i) {
                          for (
                            var h = s < t.length - 1 && -1 != t[s + 1] ? t[s + 1] : 0,
                              d = e.U.glyphToPath(r, i),
                              f = 0;
                            f < d.crds.length;
                            f += 2
                          )
                            n.crds.push(d.crds[f] + o), n.crds.push(d.crds[f + 1]);
                          for (a && n.cmds.push(a), f = 0; f < d.cmds.length; f++)
                            n.cmds.push(d.cmds[f]);
                          a && n.cmds.push('X'),
                            (o += r.hmtx.aWidth[i]),
                            s < t.length - 1 && (o += e.U.getPairAdjustment(r, i, h));
                        }
                      }
                      return n;
                    }),
                    (e.U.P = {}),
                    (e.U.P.moveTo = function (r, e, t) {
                      r.cmds.push('M'), r.crds.push(e, t);
                    }),
                    (e.U.P.lineTo = function (r, e, t) {
                      r.cmds.push('L'), r.crds.push(e, t);
                    }),
                    (e.U.P.curveTo = function (r, e, t, a, n, o, s) {
                      r.cmds.push('C'), r.crds.push(e, t, a, n, o, s);
                    }),
                    (e.U.P.qcurveTo = function (r, e, t, a, n) {
                      r.cmds.push('Q'), r.crds.push(e, t, a, n);
                    }),
                    (e.U.P.closePath = function (r) {
                      r.cmds.push('Z');
                    }),
                    (e.U._drawCFF = function (r, t, a, n, o) {
                      for (
                        var s = t.stack,
                          i = t.nStems,
                          h = t.haveWidth,
                          d = t.width,
                          f = t.open,
                          u = 0,
                          l = t.x,
                          v = t.y,
                          c = 0,
                          p = 0,
                          U = 0,
                          g = 0,
                          S = 0,
                          m = 0,
                          b = 0,
                          y = 0,
                          F = 0,
                          C = 0,
                          _ = { val: 0, size: 0 };
                        u < r.length;

                      ) {
                        e.CFF.getCharString(r, u, _);
                        var P = _.val;
                        if (((u += _.size), 'o1' == P || 'o18' == P))
                          s.length % 2 != 0 && !h && (d = s.shift() + n.nominalWidthX),
                            (i += s.length >> 1),
                            (s.length = 0),
                            (h = !0);
                        else if ('o3' == P || 'o23' == P)
                          s.length % 2 != 0 && !h && (d = s.shift() + n.nominalWidthX),
                            (i += s.length >> 1),
                            (s.length = 0),
                            (h = !0);
                        else if ('o4' == P)
                          s.length > 1 && !h && ((d = s.shift() + n.nominalWidthX), (h = !0)),
                            f && e.U.P.closePath(o),
                            (v += s.pop()),
                            e.U.P.moveTo(o, l, v),
                            (f = !0);
                        else if ('o5' == P)
                          for (; s.length > 0; )
                            (l += s.shift()), (v += s.shift()), e.U.P.lineTo(o, l, v);
                        else if ('o6' == P || 'o7' == P)
                          for (var x = s.length, I = 'o6' == P, w = 0; w < x; w++) {
                            var k = s.shift();
                            I ? (l += k) : (v += k), (I = !I), e.U.P.lineTo(o, l, v);
                          }
                        else if ('o8' == P || 'o24' == P) {
                          x = s.length;
                          for (var G = 0; G + 6 <= x; )
                            (c = l + s.shift()),
                              (p = v + s.shift()),
                              (U = c + s.shift()),
                              (g = p + s.shift()),
                              (l = U + s.shift()),
                              (v = g + s.shift()),
                              e.U.P.curveTo(o, c, p, U, g, l, v),
                              (G += 6);
                          'o24' == P && ((l += s.shift()), (v += s.shift()), e.U.P.lineTo(o, l, v));
                        } else {
                          if ('o11' == P) break;
                          if ('o1234' == P || 'o1235' == P || 'o1236' == P || 'o1237' == P)
                            'o1234' == P &&
                              ((p = v),
                              (U = (c = l + s.shift()) + s.shift()),
                              (C = g = p + s.shift()),
                              (m = g),
                              (y = v),
                              (l =
                                (b = (S = (F = U + s.shift()) + s.shift()) + s.shift()) +
                                s.shift()),
                              e.U.P.curveTo(o, c, p, U, g, F, C),
                              e.U.P.curveTo(o, S, m, b, y, l, v)),
                              'o1235' == P &&
                                ((c = l + s.shift()),
                                (p = v + s.shift()),
                                (U = c + s.shift()),
                                (g = p + s.shift()),
                                (F = U + s.shift()),
                                (C = g + s.shift()),
                                (S = F + s.shift()),
                                (m = C + s.shift()),
                                (b = S + s.shift()),
                                (y = m + s.shift()),
                                (l = b + s.shift()),
                                (v = y + s.shift()),
                                s.shift(),
                                e.U.P.curveTo(o, c, p, U, g, F, C),
                                e.U.P.curveTo(o, S, m, b, y, l, v)),
                              'o1236' == P &&
                                ((c = l + s.shift()),
                                (p = v + s.shift()),
                                (U = c + s.shift()),
                                (C = g = p + s.shift()),
                                (m = g),
                                (b = (S = (F = U + s.shift()) + s.shift()) + s.shift()),
                                (y = m + s.shift()),
                                (l = b + s.shift()),
                                e.U.P.curveTo(o, c, p, U, g, F, C),
                                e.U.P.curveTo(o, S, m, b, y, l, v)),
                              'o1237' == P &&
                                ((c = l + s.shift()),
                                (p = v + s.shift()),
                                (U = c + s.shift()),
                                (g = p + s.shift()),
                                (F = U + s.shift()),
                                (C = g + s.shift()),
                                (S = F + s.shift()),
                                (m = C + s.shift()),
                                (b = S + s.shift()),
                                (y = m + s.shift()),
                                Math.abs(b - l) > Math.abs(y - v)
                                  ? (l = b + s.shift())
                                  : (v = y + s.shift()),
                                e.U.P.curveTo(o, c, p, U, g, F, C),
                                e.U.P.curveTo(o, S, m, b, y, l, v));
                          else if ('o14' == P) {
                            if (
                              (s.length > 0 && !h && ((d = s.shift() + a.nominalWidthX), (h = !0)),
                              4 == s.length)
                            ) {
                              var O = s.shift(),
                                T = s.shift(),
                                D = s.shift(),
                                B = s.shift(),
                                A = e.CFF.glyphBySE(a, D),
                                R = e.CFF.glyphBySE(a, B);
                              e.U._drawCFF(a.CharStrings[A], t, a, n, o),
                                (t.x = O),
                                (t.y = T),
                                e.U._drawCFF(a.CharStrings[R], t, a, n, o);
                            }
                            f && (e.U.P.closePath(o), (f = !1));
                          } else if ('o19' == P || 'o20' == P)
                            s.length % 2 != 0 && !h && (d = s.shift() + n.nominalWidthX),
                              (i += s.length >> 1),
                              (s.length = 0),
                              (h = !0),
                              (u += (i + 7) >> 3);
                          else if ('o21' == P)
                            s.length > 2 && !h && ((d = s.shift() + n.nominalWidthX), (h = !0)),
                              (v += s.pop()),
                              (l += s.pop()),
                              f && e.U.P.closePath(o),
                              e.U.P.moveTo(o, l, v),
                              (f = !0);
                          else if ('o22' == P)
                            s.length > 1 && !h && ((d = s.shift() + n.nominalWidthX), (h = !0)),
                              (l += s.pop()),
                              f && e.U.P.closePath(o),
                              e.U.P.moveTo(o, l, v),
                              (f = !0);
                          else if ('o25' == P) {
                            for (; s.length > 6; )
                              (l += s.shift()), (v += s.shift()), e.U.P.lineTo(o, l, v);
                            (c = l + s.shift()),
                              (p = v + s.shift()),
                              (U = c + s.shift()),
                              (g = p + s.shift()),
                              (l = U + s.shift()),
                              (v = g + s.shift()),
                              e.U.P.curveTo(o, c, p, U, g, l, v);
                          } else if ('o26' == P)
                            for (s.length % 2 && (l += s.shift()); s.length > 0; )
                              (c = l),
                                (p = v + s.shift()),
                                (l = U = c + s.shift()),
                                (v = (g = p + s.shift()) + s.shift()),
                                e.U.P.curveTo(o, c, p, U, g, l, v);
                          else if ('o27' == P)
                            for (s.length % 2 && (v += s.shift()); s.length > 0; )
                              (p = v),
                                (U = (c = l + s.shift()) + s.shift()),
                                (g = p + s.shift()),
                                (l = U + s.shift()),
                                (v = g),
                                e.U.P.curveTo(o, c, p, U, g, l, v);
                          else if ('o10' == P || 'o29' == P) {
                            var L = 'o10' == P ? n : a;
                            if (0 == s.length) console.debug('error: empty stack');
                            else {
                              var W = s.pop(),
                                M = L.Subrs[W + L.Bias];
                              (t.x = l),
                                (t.y = v),
                                (t.nStems = i),
                                (t.haveWidth = h),
                                (t.width = d),
                                (t.open = f),
                                e.U._drawCFF(M, t, a, n, o),
                                (l = t.x),
                                (v = t.y),
                                (i = t.nStems),
                                (h = t.haveWidth),
                                (d = t.width),
                                (f = t.open);
                            }
                          } else if ('o30' == P || 'o31' == P) {
                            var V = s.length,
                              E = ((G = 0), 'o31' == P);
                            for (G += V - (x = -3 & V); G < x; )
                              E
                                ? ((p = v),
                                  (U = (c = l + s.shift()) + s.shift()),
                                  (v = (g = p + s.shift()) + s.shift()),
                                  x - G == 5 ? ((l = U + s.shift()), G++) : (l = U),
                                  (E = !1))
                                : ((c = l),
                                  (p = v + s.shift()),
                                  (U = c + s.shift()),
                                  (g = p + s.shift()),
                                  (l = U + s.shift()),
                                  x - G == 5 ? ((v = g + s.shift()), G++) : (v = g),
                                  (E = !0)),
                                e.U.P.curveTo(o, c, p, U, g, l, v),
                                (G += 4);
                          } else {
                            if ('o' == (P + '').charAt(0))
                              throw (console.debug('Unknown operation: ' + P, r), P);
                            s.push(P);
                          }
                        }
                      }
                      (t.x = l),
                        (t.y = v),
                        (t.nStems = i),
                        (t.haveWidth = h),
                        (t.width = d),
                        (t.open = f);
                    });
                  var t = e,
                    a = { Typr: t };
                  return (
                    (r.Typr = t),
                    (r.default = a),
                    Object.defineProperty(r, '__esModule', { value: !0 }),
                    r
                  );
                })({}).Typr
              );
            },
            function woff2otfFactory() {
              return (function (r) {
                var e = Uint8Array,
                  n = Uint16Array,
                  t = Uint32Array,
                  a = new e([
                    0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5,
                    5, 0, 0, 0, 0,
                  ]),
                  i = new e([
                    0, 0, 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 10, 10, 11,
                    11, 12, 12, 13, 13, 0, 0,
                  ]),
                  o = new e([16, 17, 18, 0, 8, 7, 9, 6, 10, 5, 11, 4, 12, 3, 13, 2, 14, 1, 15]),
                  f = function (r, e) {
                    for (var a = new n(31), i = 0; i < 31; ++i) a[i] = e += 1 << r[i - 1];
                    var o = new t(a[30]);
                    for (i = 1; i < 30; ++i)
                      for (var f = a[i]; f < a[i + 1]; ++f) o[f] = ((f - a[i]) << 5) | i;
                    return [a, o];
                  },
                  u = f(a, 2),
                  v = u[0],
                  s = u[1];
                (v[28] = 258), (s[258] = 28);
                for (var l = f(i, 0)[0], c = new n(32768), g = 0; g < 32768; ++g) {
                  var h = ((43690 & g) >>> 1) | ((21845 & g) << 1);
                  (h =
                    ((61680 & (h = ((52428 & h) >>> 2) | ((13107 & h) << 2))) >>> 4) |
                    ((3855 & h) << 4)),
                    (c[g] = (((65280 & h) >>> 8) | ((255 & h) << 8)) >>> 1);
                }
                var w = function (r, e, t) {
                    for (var a = r.length, i = 0, o = new n(e); i < a; ++i) ++o[r[i] - 1];
                    var f,
                      u = new n(e);
                    for (i = 0; i < e; ++i) u[i] = (u[i - 1] + o[i - 1]) << 1;
                    if (t) {
                      f = new n(1 << e);
                      var v = 15 - e;
                      for (i = 0; i < a; ++i)
                        if (r[i])
                          for (
                            var s = (i << 4) | r[i],
                              l = e - r[i],
                              g = u[r[i] - 1]++ << l,
                              h = g | ((1 << l) - 1);
                            g <= h;
                            ++g
                          )
                            f[c[g] >>> v] = s;
                    } else
                      for (f = new n(a), i = 0; i < a; ++i)
                        r[i] && (f[i] = c[u[r[i] - 1]++] >>> (15 - r[i]));
                    return f;
                  },
                  d = new e(288);
                for (g = 0; g < 144; ++g) d[g] = 8;
                for (g = 144; g < 256; ++g) d[g] = 9;
                for (g = 256; g < 280; ++g) d[g] = 7;
                for (g = 280; g < 288; ++g) d[g] = 8;
                var m = new e(32);
                for (g = 0; g < 32; ++g) m[g] = 5;
                var b = w(d, 9, 1),
                  p = w(m, 5, 1),
                  y = function (r) {
                    for (var e = r[0], n = 1; n < r.length; ++n) r[n] > e && (e = r[n]);
                    return e;
                  },
                  L = function (r, e, n) {
                    var t = (e / 8) | 0;
                    return ((r[t] | (r[t + 1] << 8)) >> (7 & e)) & n;
                  },
                  U = function (r, e) {
                    var n = (e / 8) | 0;
                    return (r[n] | (r[n + 1] << 8) | (r[n + 2] << 16)) >> (7 & e);
                  },
                  k = [
                    'unexpected EOF',
                    'invalid block type',
                    'invalid length/literal',
                    'invalid distance',
                    'stream finished',
                    'no stream handler',
                    ,
                    'no callback',
                    'invalid UTF-8 data',
                    'extra field too long',
                    'date not in range 1980-2099',
                    'filename too long',
                    'stream finishing',
                    'invalid zip data',
                  ],
                  T = function (r, e, n) {
                    var t = new Error(e || k[r]);
                    if (
                      ((t.code = r), Error.captureStackTrace && Error.captureStackTrace(t, T), !n)
                    )
                      throw t;
                    return t;
                  },
                  O = function (r, f, u) {
                    var s = r.length;
                    if (!s || (u && !u.l && s < 5)) return f || new e(0);
                    var c = !f || u,
                      g = !u || u.i;
                    u || (u = {}), f || (f = new e(3 * s));
                    var h,
                      d = function (r) {
                        var n = f.length;
                        if (r > n) {
                          var t = new e(Math.max(2 * n, r));
                          t.set(f), (f = t);
                        }
                      },
                      m = u.f || 0,
                      k = u.p || 0,
                      O = u.b || 0,
                      A = u.l,
                      x = u.d,
                      E = u.m,
                      D = u.n,
                      M = 8 * s;
                    do {
                      if (!A) {
                        u.f = m = L(r, k, 1);
                        var S = L(r, k + 1, 3);
                        if (((k += 3), !S)) {
                          var V =
                              r[(I = (((h = k) / 8) | 0) + (7 & h && 1) + 4) - 4] | (r[I - 3] << 8),
                            _ = I + V;
                          if (_ > s) {
                            g && T(0);
                            break;
                          }
                          c && d(O + V),
                            f.set(r.subarray(I, _), O),
                            (u.b = O += V),
                            (u.p = k = 8 * _);
                          continue;
                        }
                        if (1 == S) (A = b), (x = p), (E = 9), (D = 5);
                        else if (2 == S) {
                          var j = L(r, k, 31) + 257,
                            z = L(r, k + 10, 15) + 4,
                            C = j + L(r, k + 5, 31) + 1;
                          k += 14;
                          for (var F = new e(C), P = new e(19), q = 0; q < z; ++q)
                            P[o[q]] = L(r, k + 3 * q, 7);
                          k += 3 * z;
                          var B = y(P),
                            G = (1 << B) - 1,
                            H = w(P, B, 1);
                          for (q = 0; q < C; ) {
                            var I,
                              J = H[L(r, k, G)];
                            if (((k += 15 & J), (I = J >>> 4) < 16)) F[q++] = I;
                            else {
                              var K = 0,
                                N = 0;
                              for (
                                16 == I
                                  ? ((N = 3 + L(r, k, 3)), (k += 2), (K = F[q - 1]))
                                  : 17 == I
                                    ? ((N = 3 + L(r, k, 7)), (k += 3))
                                    : 18 == I && ((N = 11 + L(r, k, 127)), (k += 7));
                                N--;

                              )
                                F[q++] = K;
                            }
                          }
                          var Q = F.subarray(0, j),
                            R = F.subarray(j);
                          (E = y(Q)), (D = y(R)), (A = w(Q, E, 1)), (x = w(R, D, 1));
                        } else T(1);
                        if (k > M) {
                          g && T(0);
                          break;
                        }
                      }
                      c && d(O + 131072);
                      for (var W = (1 << E) - 1, X = (1 << D) - 1, Y = k; ; Y = k) {
                        var Z = (K = A[U(r, k) & W]) >>> 4;
                        if ((k += 15 & K) > M) {
                          g && T(0);
                          break;
                        }
                        if ((K || T(2), Z < 256)) f[O++] = Z;
                        else {
                          if (256 == Z) {
                            (Y = k), (A = null);
                            break;
                          }
                          var $ = Z - 254;
                          if (Z > 264) {
                            var rr = a[(q = Z - 257)];
                            ($ = L(r, k, (1 << rr) - 1) + v[q]), (k += rr);
                          }
                          var er = x[U(r, k) & X],
                            nr = er >>> 4;
                          if (
                            (er || T(3),
                            (k += 15 & er),
                            (R = l[nr]),
                            nr > 3 && ((rr = i[nr]), (R += U(r, k) & ((1 << rr) - 1)), (k += rr)),
                            k > M)
                          ) {
                            g && T(0);
                            break;
                          }
                          c && d(O + 131072);
                          for (var tr = O + $; O < tr; O += 4)
                            (f[O] = f[O - R]),
                              (f[O + 1] = f[O + 1 - R]),
                              (f[O + 2] = f[O + 2 - R]),
                              (f[O + 3] = f[O + 3 - R]);
                          O = tr;
                        }
                      }
                      (u.l = A),
                        (u.p = Y),
                        (u.b = O),
                        A && ((m = 1), (u.m = E), (u.d = x), (u.n = D));
                    } while (!m);
                    return O == f.length
                      ? f
                      : (function (r, a, i) {
                          (null == a || a < 0) && (a = 0),
                            (null == i || i > r.length) && (i = r.length);
                          var o = new (r instanceof n ? n : r instanceof t ? t : e)(i - a);
                          return o.set(r.subarray(a, i)), o;
                        })(f, 0, O);
                  },
                  A = new e(0),
                  x = 'undefined' != typeof TextDecoder && new TextDecoder();
                try {
                  x.decode(A, { stream: !0 });
                } catch (r) {}
                return (
                  (r.convert_streams = function (r) {
                    var e = new DataView(r),
                      n = 0;
                    function t() {
                      var r = e.getUint16(n);
                      return (n += 2), r;
                    }
                    function a() {
                      var r = e.getUint32(n);
                      return (n += 4), r;
                    }
                    function i(r) {
                      m.setUint16(b, r), (b += 2);
                    }
                    function o(r) {
                      m.setUint32(b, r), (b += 4);
                    }
                    for (
                      var f = {
                          signature: a(),
                          flavor: a(),
                          length: a(),
                          numTables: t(),
                          reserved: t(),
                          totalSfntSize: a(),
                          majorVersion: t(),
                          minorVersion: t(),
                          metaOffset: a(),
                          metaLength: a(),
                          metaOrigLength: a(),
                          privOffset: a(),
                          privLength: a(),
                        },
                        u = 0;
                      Math.pow(2, u) <= f.numTables;

                    )
                      u++;
                    u--;
                    for (
                      var v = 16 * Math.pow(2, u), s = 16 * f.numTables - v, l = 12, c = [], g = 0;
                      g < f.numTables;
                      g++
                    )
                      c.push({
                        tag: a(),
                        offset: a(),
                        compLength: a(),
                        origLength: a(),
                        origChecksum: a(),
                      }),
                        (l += 16);
                    var h,
                      w = new Uint8Array(
                        12 +
                          16 * c.length +
                          c.reduce(function (r, e) {
                            return r + e.origLength + 4;
                          }, 0),
                      ),
                      d = w.buffer,
                      m = new DataView(d),
                      b = 0;
                    return (
                      o(f.flavor),
                      i(f.numTables),
                      i(v),
                      i(u),
                      i(s),
                      c.forEach(function (r) {
                        o(r.tag),
                          o(r.origChecksum),
                          o(l),
                          o(r.origLength),
                          (r.outOffset = l),
                          (l += r.origLength) % 4 != 0 && (l += 4 - (l % 4));
                      }),
                      c.forEach(function (e) {
                        var n,
                          t = r.slice(e.offset, e.offset + e.compLength);
                        if (e.compLength != e.origLength) {
                          var a = new Uint8Array(e.origLength);
                          (n = new Uint8Array(t, 2)), O(n, a);
                        } else a = new Uint8Array(t);
                        w.set(a, e.outOffset);
                        var i = 0;
                        (l = e.outOffset + e.origLength) % 4 != 0 && (i = 4 - (l % 4)),
                          w.set(new Uint8Array(i).buffer, e.outOffset + e.origLength),
                          (h = l + i);
                      }),
                      d.slice(0, h)
                    );
                  }),
                  Object.defineProperty(r, '__esModule', { value: !0 }),
                  r
                );
              })({}).convert_streams;
            },
            function parserFactory(Typr, woff2otf) {
              const cmdArgLengths = { M: 2, L: 2, Q: 4, C: 6, Z: 0 },
                joiningTypeRawData = {
                  C: '18g,ca,368,1kz',
                  D: '17k,6,2,2+4,5+c,2+6,2+1,10+1,9+f,j+11,2+1,a,2,2+1,15+2,3,j+2,6+3,2+8,2,2,2+1,w+a,4+e,3+3,2,3+2,3+5,23+w,2f+4,3,2+9,2,b,2+3,3,1k+9,6+1,3+1,2+2,2+d,30g,p+y,1,1+1g,f+x,2,sd2+1d,jf3+4,f+3,2+4,2+2,b+3,42,2,4+2,2+1,2,3,t+1,9f+w,2,el+2,2+g,d+2,2l,2+1,5,3+1,2+1,2,3,6,16wm+1v',
                  R: '17m+3,2,2,6+3,m,15+2,2+2,h+h,13,3+8,2,2,3+1,2,p+1,x,5+4,5,a,2,2,3,u,c+2,g+1,5,2+1,4+1,5j,6+1,2,b,2+2,f,2+1,1s+2,2,3+1,7,1ez0,2,2+1,4+4,b,4,3,b,42,2+2,4,3,2+1,2,o+3,ae,ep,x,2o+2,3+1,3,5+1,6',
                  L: 'x9u,jff,a,fd,jv',
                  T: '4t,gj+33,7o+4,1+1,7c+18,2,2+1,2+1,2,21+a,2,1b+k,h,2u+6,3+5,3+1,2+3,y,2,v+q,2k+a,1n+8,a,p+3,2+8,2+2,2+4,18+2,3c+e,2+v,1k,2,5+7,5,4+6,b+1,u,1n,5+3,9,l+1,r,3+1,1m,5+1,5+1,3+2,4,v+1,4,c+1,1m,5+4,2+1,5,l+1,n+5,2,1n,3,2+3,9,8+1,c+1,v,1q,d,1f,4,1m+2,6+2,2+3,8+1,c+1,u,1n,3,7,6+1,l+1,t+1,1m+1,5+3,9,l+1,u,21,8+2,2,2j,3+6,d+7,2r,3+8,c+5,23+1,s,2,2,1k+d,2+4,2+1,6+a,2+z,a,2v+3,2+5,2+1,3+1,q+1,5+2,h+3,e,3+1,7,g,jk+2,qb+2,u+2,u+1,v+1,1t+1,2+6,9,3+a,a,1a+2,3c+1,z,3b+2,5+1,a,7+2,64+1,3,1n,2+6,2,2,3+7,7+9,3,1d+d,1,1+1,1s+3,1d,2+4,2,6,15+8,d+1,x+3,3+1,2+2,1l,2+1,4,2+2,1n+7,3+1,49+2,2+c,2+6,5,7,4+1,5j+1l,2+4,ek,3+1,r+4,1e+4,6+5,2p+c,1+3,1,1+2,1+b,2db+2,3y,2p+v,ff+3,30+1,n9x,1+2,2+9,x+1,29+1,7l,4,5,q+1,6,48+1,r+h,e,13+7,q+a,1b+2,1d,3+3,3+1,14,1w+5,3+1,3+1,d,9,1c,1g,2+2,3+1,6+1,2,17+1,9,6n,3,5,fn5,ki+f,h+f,5s,6y+2,ea,6b,46+4,1af+2,2+1,6+3,15+2,5,4m+1,fy+3,as+1,4a+a,4x,1j+e,1l+2,1e+3,3+1,1y+2,11+4,2+7,1r,d+1,1h+8,b+3,3,2o+2,3,2+1,7,4h,4+7,m+1,1m+1,4,12+6,4+4,5g+7,3+2,2,o,2d+5,2,5+1,2+1,6n+3,7+1,2+1,s+1,2e+7,3,2+1,2z,2,3+5,2,2u+2,3+3,2+4,78+8,2+1,75+1,2,5,41+3,3+1,5,x+9,15+5,3+3,9,a+5,3+2,1b+c,2+1,bb+6,2+5,2,2b+l,3+6,2+1,2+1,3f+5,4,2+1,2+6,2,21+1,4,2,9o+1,470+8,at4+4,1o+6,t5,1s+3,2a,f5l+1,2+3,43o+2,a+7,1+7,3+6,v+3,45+2,1j0+1i,5+1d,9,f,n+4,2+e,11t+6,2+g,3+6,2+1,2+4,7a+6,c6+3,15t+6,32+6,1,gzau,v+2n,3l+6n',
                },
                JT_LEFT = 1,
                JT_RIGHT = 2,
                JT_DUAL = 4,
                JT_TRANSPARENT = 8,
                JT_JOIN_CAUSING = 16,
                JT_NON_JOINING = 32;
              let joiningTypeMap;
              function getCharJoiningType(ch) {
                if (!joiningTypeMap) {
                  const m = {
                    R: JT_RIGHT,
                    L: JT_LEFT,
                    D: JT_DUAL,
                    C: JT_JOIN_CAUSING,
                    U: JT_NON_JOINING,
                    T: JT_TRANSPARENT,
                  };
                  joiningTypeMap = new Map();
                  for (let type in joiningTypeRawData) {
                    let lastCode = 0;
                    joiningTypeRawData[type].split(',').forEach((range) => {
                      let [skip, step] = range.split('+');
                      (skip = parseInt(skip, 36)),
                        (step = step ? parseInt(step, 36) : 0),
                        joiningTypeMap.set((lastCode += skip), m[type]);
                      for (let i = step; i--; ) joiningTypeMap.set(++lastCode, m[type]);
                    });
                  }
                }
                return joiningTypeMap.get(ch) || JT_NON_JOINING;
              }
              const ISOL = 1,
                INIT = 2,
                FINA = 3,
                MEDI = 4,
                formsToFeatures = [null, 'isol', 'init', 'fina', 'medi'];
              function detectJoiningForms(str) {
                const joiningForms = new Uint8Array(str.length);
                let prevJoiningType = JT_NON_JOINING,
                  prevForm = ISOL,
                  prevIndex = -1;
                for (let i = 0; i < str.length; i++) {
                  const code = str.codePointAt(i);
                  let joiningType = 0 | getCharJoiningType(code),
                    form = ISOL;
                  joiningType & JT_TRANSPARENT ||
                    (prevJoiningType & (JT_LEFT | JT_DUAL | JT_JOIN_CAUSING)
                      ? joiningType & (JT_RIGHT | JT_DUAL | JT_JOIN_CAUSING)
                        ? ((form = FINA),
                          (prevForm !== ISOL && prevForm !== FINA) || joiningForms[prevIndex]++)
                        : joiningType & (JT_LEFT | JT_NON_JOINING) &&
                          ((prevForm !== INIT && prevForm !== MEDI) || joiningForms[prevIndex]--)
                      : prevJoiningType & (JT_RIGHT | JT_NON_JOINING) &&
                        ((prevForm !== INIT && prevForm !== MEDI) || joiningForms[prevIndex]--),
                    (prevForm = joiningForms[i] = form),
                    (prevJoiningType = joiningType),
                    (prevIndex = i),
                    code > 65535 && i++);
                }
                return joiningForms;
              }
              function getGlyphClass(font, glyphId) {
                const classDef = font.GDEF && font.GDEF.glyphClassDef;
                return classDef ? Typr.U._getGlyphClass(glyphId, classDef) : 0;
              }
              function firstNum(...args) {
                for (let i = 0; i < args.length; i++)
                  if ('number' == typeof args[i]) return args[i];
              }
              function wrapFontObj(typrFont) {
                const glyphMap = Object.create(null),
                  os2 = typrFont['OS/2'],
                  hhea = typrFont.hhea,
                  unitsPerEm = typrFont.head.unitsPerEm,
                  ascender = firstNum(os2 && os2.sTypoAscender, hhea && hhea.ascender, unitsPerEm),
                  fontObj = {
                    unitsPerEm,
                    ascender,
                    descender: firstNum(os2 && os2.sTypoDescender, hhea && hhea.descender, 0),
                    capHeight: firstNum(os2 && os2.sCapHeight, ascender),
                    xHeight: firstNum(os2 && os2.sxHeight, ascender),
                    lineGap: firstNum(os2 && os2.sTypoLineGap, hhea && hhea.lineGap),
                    supportsCodePoint: (code) => Typr.U.codeToGlyph(typrFont, code) > 0,
                    forEachGlyph(text, fontSize, letterSpacing, callback) {
                      let penX = 0;
                      const fontScale = (1 / fontObj.unitsPerEm) * fontSize,
                        glyphIds = (function stringToGlyphs(font, str) {
                          const glyphIds = [];
                          for (let i = 0; i < str.length; i++) {
                            const cc = str.codePointAt(i);
                            cc > 65535 && i++, glyphIds.push(Typr.U.codeToGlyph(font, cc));
                          }
                          const gsub = font.GSUB;
                          if (gsub) {
                            const { lookupList, featureList } = gsub;
                            let joiningForms;
                            const supportedFeatures =
                                /^(rlig|liga|mset|isol|init|fina|medi|half|pres|blws|ccmp)$/,
                              usedLookups = [];
                            featureList.forEach((feature) => {
                              if (supportedFeatures.test(feature.tag))
                                for (let ti = 0; ti < feature.tab.length; ti++) {
                                  if (usedLookups[feature.tab[ti]]) continue;
                                  usedLookups[feature.tab[ti]] = !0;
                                  const tab = lookupList[feature.tab[ti]],
                                    isJoiningFeature = /^(isol|init|fina|medi)$/.test(feature.tag);
                                  isJoiningFeature &&
                                    !joiningForms &&
                                    (joiningForms = detectJoiningForms(str));
                                  for (let ci = 0; ci < glyphIds.length; ci++)
                                    (joiningForms &&
                                      isJoiningFeature &&
                                      formsToFeatures[joiningForms[ci]] !== feature.tag) ||
                                      Typr.U._applySubs(glyphIds, ci, tab, lookupList);
                                }
                            });
                          }
                          return glyphIds;
                        })(typrFont, text);
                      let charIndex = 0;
                      const positions = (function calcGlyphPositions(font, glyphIds) {
                        const positions = new Int16Array(3 * glyphIds.length);
                        let glyphIndex = 0;
                        for (; glyphIndex < glyphIds.length; glyphIndex++) {
                          const glyphId = glyphIds[glyphIndex];
                          if (-1 === glyphId) continue;
                          positions[3 * glyphIndex + 2] = font.hmtx.aWidth[glyphId];
                          const gpos = font.GPOS;
                          if (gpos) {
                            const llist = gpos.lookupList;
                            for (let i = 0; i < llist.length; i++) {
                              const lookup = llist[i];
                              for (let j = 0; j < lookup.tabs.length; j++) {
                                const tab = lookup.tabs[j];
                                if (1 === lookup.ltype) {
                                  if (
                                    -1 !== Typr._lctf.coverageIndex(tab.coverage, glyphId) &&
                                    tab.pos
                                  ) {
                                    applyValueRecord(tab.pos, glyphIndex);
                                    break;
                                  }
                                } else if (2 === lookup.ltype) {
                                  let adj = null,
                                    prevGlyphIndex = getPrevGlyphIndex();
                                  if (-1 !== prevGlyphIndex) {
                                    const coverageIndex = Typr._lctf.coverageIndex(
                                      tab.coverage,
                                      glyphIds[prevGlyphIndex],
                                    );
                                    if (-1 !== coverageIndex) {
                                      if (1 === tab.fmt) {
                                        const right = tab.pairsets[coverageIndex];
                                        for (let k = 0; k < right.length; k++)
                                          right[k].gid2 === glyphId && (adj = right[k]);
                                      } else if (2 === tab.fmt) {
                                        const c1 = Typr.U._getGlyphClass(
                                            glyphIds[prevGlyphIndex],
                                            tab.classDef1,
                                          ),
                                          c2 = Typr.U._getGlyphClass(glyphId, tab.classDef2);
                                        adj = tab.matrix[c1][c2];
                                      }
                                      if (adj) {
                                        adj.val1 && applyValueRecord(adj.val1, prevGlyphIndex),
                                          adj.val2 && applyValueRecord(adj.val2, glyphIndex);
                                        break;
                                      }
                                    }
                                  }
                                } else if (4 === lookup.ltype) {
                                  const markArrIndex = Typr._lctf.coverageIndex(
                                    tab.markCoverage,
                                    glyphId,
                                  );
                                  if (-1 !== markArrIndex) {
                                    const baseGlyphIndex = getPrevGlyphIndex(isBaseGlyph),
                                      baseArrIndex =
                                        -1 === baseGlyphIndex
                                          ? -1
                                          : Typr._lctf.coverageIndex(
                                              tab.baseCoverage,
                                              glyphIds[baseGlyphIndex],
                                            );
                                    if (-1 !== baseArrIndex) {
                                      const markRecord = tab.markArray[markArrIndex],
                                        baseAnchor =
                                          tab.baseArray[baseArrIndex][markRecord.markClass];
                                      (positions[3 * glyphIndex] =
                                        baseAnchor.x -
                                        markRecord.x +
                                        positions[3 * baseGlyphIndex] -
                                        positions[3 * baseGlyphIndex + 2]),
                                        (positions[3 * glyphIndex + 1] =
                                          baseAnchor.y -
                                          markRecord.y +
                                          positions[3 * baseGlyphIndex + 1]);
                                      break;
                                    }
                                  }
                                } else if (6 === lookup.ltype) {
                                  const mark1ArrIndex = Typr._lctf.coverageIndex(
                                    tab.mark1Coverage,
                                    glyphId,
                                  );
                                  if (-1 !== mark1ArrIndex) {
                                    const prevGlyphIndex = getPrevGlyphIndex();
                                    if (-1 !== prevGlyphIndex) {
                                      const prevGlyphId = glyphIds[prevGlyphIndex];
                                      if (3 === getGlyphClass(font, prevGlyphId)) {
                                        const mark2ArrIndex = Typr._lctf.coverageIndex(
                                          tab.mark2Coverage,
                                          prevGlyphId,
                                        );
                                        if (-1 !== mark2ArrIndex) {
                                          const mark1Record = tab.mark1Array[mark1ArrIndex],
                                            mark2Anchor =
                                              tab.mark2Array[mark2ArrIndex][mark1Record.markClass];
                                          (positions[3 * glyphIndex] =
                                            mark2Anchor.x -
                                            mark1Record.x +
                                            positions[3 * prevGlyphIndex] -
                                            positions[3 * prevGlyphIndex + 2]),
                                            (positions[3 * glyphIndex + 1] =
                                              mark2Anchor.y -
                                              mark1Record.y +
                                              positions[3 * prevGlyphIndex + 1]);
                                          break;
                                        }
                                      }
                                    }
                                  }
                                }
                              }
                            }
                          } else if (font.kern && !font.cff) {
                            const prevGlyphIndex = getPrevGlyphIndex();
                            if (-1 !== prevGlyphIndex) {
                              const ind1 = font.kern.glyph1.indexOf(glyphIds[prevGlyphIndex]);
                              if (-1 !== ind1) {
                                const ind2 = font.kern.rval[ind1].glyph2.indexOf(glyphId);
                                -1 !== ind2 &&
                                  (positions[3 * prevGlyphIndex + 2] +=
                                    font.kern.rval[ind1].vals[ind2]);
                              }
                            }
                          }
                        }
                        return positions;
                        function getPrevGlyphIndex(filter) {
                          for (let i = glyphIndex - 1; i >= 0; i--)
                            if (-1 !== glyphIds[i] && (!filter || filter(glyphIds[i]))) return i;
                          return -1;
                        }
                        function isBaseGlyph(glyphId) {
                          return 1 === getGlyphClass(font, glyphId);
                        }
                        function applyValueRecord(source, gi) {
                          for (let i = 0; i < 3; i++) positions[3 * gi + i] += source[i] || 0;
                        }
                      })(typrFont, glyphIds);
                      return (
                        glyphIds.forEach((glyphId, i) => {
                          if (-1 !== glyphId) {
                            let glyphObj = glyphMap[glyphId];
                            if (!glyphObj) {
                              const { cmds, crds } = Typr.U.glyphToPath(typrFont, glyphId);
                              let xMin,
                                yMin,
                                xMax,
                                yMax,
                                path = '',
                                crdsIdx = 0;
                              for (let i = 0, len = cmds.length; i < len; i++) {
                                const numArgs = cmdArgLengths[cmds[i]];
                                path += cmds[i];
                                for (let j = 1; j <= numArgs; j++)
                                  path += (j > 1 ? ',' : '') + crds[crdsIdx++];
                              }
                              if (crds.length) {
                                (xMin = yMin = 1 / 0), (xMax = yMax = -1 / 0);
                                for (let i = 0, len = crds.length; i < len; i += 2) {
                                  let x = crds[i],
                                    y = crds[i + 1];
                                  x < xMin && (xMin = x),
                                    y < yMin && (yMin = y),
                                    x > xMax && (xMax = x),
                                    y > yMax && (yMax = y);
                                }
                              } else xMin = xMax = yMin = yMax = 0;
                              glyphObj = glyphMap[glyphId] = {
                                index: glyphId,
                                advanceWidth: typrFont.hmtx.aWidth[glyphId],
                                xMin,
                                yMin,
                                xMax,
                                yMax,
                                path,
                              };
                            }
                            callback.call(
                              null,
                              glyphObj,
                              penX + positions[3 * i] * fontScale,
                              positions[3 * i + 1] * fontScale,
                              charIndex,
                            ),
                              (penX += positions[3 * i + 2] * fontScale),
                              letterSpacing && (penX += letterSpacing * fontSize);
                          }
                          charIndex += text.codePointAt(charIndex) > 65535 ? 2 : 1;
                        }),
                        penX
                      );
                    },
                  };
                return fontObj;
              }
              return function parse(buffer) {
                const peek = new Uint8Array(buffer, 0, 4),
                  tag = Typr._bin.readASCII(peek, 0, 4);
                if ('wOFF' === tag) buffer = woff2otf(buffer);
                else if ('wOF2' === tag) throw new Error('woff2 fonts not supported');
                return wrapFontObj(Typr.parse(buffer)[0]);
              };
            },
          ],
          init: (typrFactory, woff2otfFactory, parserFactory) =>
            parserFactory(typrFactory(), woff2otfFactory()),
        });
        const fontResolverWorkerModule = defineWorkerModule({
          name: 'FontResolver',
          dependencies: [
            function createFontResolver(fontParser, unicodeFontResolverClient) {
              const parsedFonts = Object.create(null),
                loadingFonts = Object.create(null);
              function loadFont(fontUrl, callback) {
                let font = parsedFonts[fontUrl];
                font
                  ? callback(font)
                  : loadingFonts[fontUrl]
                    ? loadingFonts[fontUrl].push(callback)
                    : ((loadingFonts[fontUrl] = [callback]),
                      (function doLoadFont(url, callback) {
                        const onError = (err) => {
                          console.error(`Failure loading font ${url}`, err);
                        };
                        try {
                          const request = new XMLHttpRequest();
                          request.open('get', url, !0),
                            (request.responseType = 'arraybuffer'),
                            (request.onload = function () {
                              if (request.status >= 400) onError(new Error(request.statusText));
                              else if (request.status > 0)
                                try {
                                  const fontObj = fontParser(request.response);
                                  (fontObj.src = url), callback(fontObj);
                                } catch (e) {
                                  onError(e);
                                }
                            }),
                            (request.onerror = onError),
                            request.send();
                        } catch (err) {
                          onError(err);
                        }
                      })(fontUrl, (fontObj) => {
                        (fontObj.src = fontUrl),
                          (parsedFonts[fontUrl] = fontObj),
                          loadingFonts[fontUrl].forEach((cb) => cb(fontObj)),
                          delete loadingFonts[fontUrl];
                      }));
              }
              return function (
                text,
                callback,
                {
                  lang,
                  fonts: userFonts = [],
                  style = 'normal',
                  weight = 'normal',
                  unicodeFontsURL,
                } = {},
              ) {
                const charResolutions = new Uint8Array(text.length),
                  fontResolutions = [];
                text.length || allDone();
                const fontIndices = new Map(),
                  fallbackRanges = [];
                if (
                  ('italic' !== style && (style = 'normal'),
                  'number' != typeof weight && (weight = 'bold' === weight ? 700 : 400),
                  userFonts && !Array.isArray(userFonts) && (userFonts = [userFonts]),
                  (userFonts = userFonts
                    .slice()
                    .filter((def) => !def.lang || def.lang.test(lang))
                    .reverse()).length)
                ) {
                  const RESOLVED = 1,
                    NEEDS_FALLBACK = 2;
                  let prevCharResult = 0;
                  !(function resolveUserFonts(startIndex = 0) {
                    for (let i = startIndex, iLen = text.length; i < iLen; i++) {
                      const codePoint = text.codePointAt(i);
                      if (
                        (prevCharResult === RESOLVED &&
                          fontResolutions[charResolutions[i - 1]].supportsCodePoint(codePoint)) ||
                        /\s/.test(text[i])
                      )
                        (charResolutions[i] = charResolutions[i - 1]),
                          prevCharResult === NEEDS_FALLBACK &&
                            (fallbackRanges[fallbackRanges.length - 1][1] = i);
                      else
                        for (let j = charResolutions[i], jLen = userFonts.length; j <= jLen; j++)
                          if (j === jLen) {
                            ((prevCharResult === NEEDS_FALLBACK
                              ? fallbackRanges[fallbackRanges.length - 1]
                              : (fallbackRanges[fallbackRanges.length] = [i, i]))[1] = i),
                              (prevCharResult = NEEDS_FALLBACK);
                          } else {
                            charResolutions[i] = j;
                            const { src, unicodeRange } = userFonts[j];
                            if (!unicodeRange || isCodeInRanges(codePoint, unicodeRange)) {
                              const fontObj = parsedFonts[src];
                              if (!fontObj)
                                return void loadFont(src, () => {
                                  resolveUserFonts(i);
                                });
                              if (fontObj.supportsCodePoint(codePoint)) {
                                let fontIndex = fontIndices.get(fontObj);
                                'number' != typeof fontIndex &&
                                  ((fontIndex = fontResolutions.length),
                                  fontResolutions.push(fontObj),
                                  fontIndices.set(fontObj, fontIndex)),
                                  (charResolutions[i] = fontIndex),
                                  (prevCharResult = RESOLVED);
                                break;
                              }
                            }
                          }
                      codePoint > 65535 &&
                        i + 1 < iLen &&
                        ((charResolutions[i + 1] = charResolutions[i]),
                        i++,
                        prevCharResult === NEEDS_FALLBACK &&
                          (fallbackRanges[fallbackRanges.length - 1][1] = i));
                    }
                    resolveFallbacks();
                  })();
                } else fallbackRanges.push([0, text.length - 1]), resolveFallbacks();
                function resolveFallbacks() {
                  if (fallbackRanges.length) {
                    const fallbackString = fallbackRanges
                      .map((range) => text.substring(range[0], range[1] + 1))
                      .join('\n');
                    unicodeFontResolverClient
                      .getFontsForString(fallbackString, {
                        lang: lang || void 0,
                        style,
                        weight,
                        dataUrl: unicodeFontsURL,
                      })
                      .then(({ fontUrls, chars }) => {
                        const fontIndexOffset = fontResolutions.length;
                        let charIdx = 0;
                        fallbackRanges.forEach((range) => {
                          for (let i = 0, endIdx = range[1] - range[0]; i <= endIdx; i++)
                            charResolutions[range[0] + i] = chars[charIdx++] + fontIndexOffset;
                          charIdx++;
                        });
                        let loadedCount = 0;
                        fontUrls.forEach((url, i) => {
                          loadFont(url, (fontObj) => {
                            (fontResolutions[i + fontIndexOffset] = fontObj),
                              ++loadedCount === fontUrls.length && allDone();
                          });
                        });
                      });
                  } else allDone();
                }
                function allDone() {
                  callback({ chars: charResolutions, fonts: fontResolutions });
                }
                function isCodeInRanges(code, ranges) {
                  for (let k = 0; k < ranges.length; k++) {
                    const [start, end = start] = ranges[k];
                    if (start <= code && code <= end) return !0;
                  }
                  return !1;
                }
              };
            },
            workerModule,
            function unicodeFontResolverClientFactory() {
              return (function (t) {
                var n = function () {
                  this.buckets = new Map();
                };
                (n.prototype.add = function (t) {
                  var n = t >> 5;
                  this.buckets.set(n, (this.buckets.get(n) || 0) | (1 << (31 & t)));
                }),
                  (n.prototype.has = function (t) {
                    var n = this.buckets.get(t >> 5);
                    return void 0 !== n && !!(n & (1 << (31 & t)));
                  }),
                  (n.prototype.serialize = function () {
                    var t = [];
                    return (
                      this.buckets.forEach(function (n, r) {
                        t.push((+r).toString(36) + ':' + n.toString(36));
                      }),
                      t.join(',')
                    );
                  }),
                  (n.prototype.deserialize = function (t) {
                    var n = this;
                    this.buckets.clear(),
                      t.split(',').forEach(function (t) {
                        var r = t.split(':');
                        n.buckets.set(parseInt(r[0], 36), parseInt(r[1], 36));
                      });
                  });
                var r = Math.pow(2, 8),
                  e = r - 1,
                  o = ~e;
                function a(t) {
                  var n = (function (t) {
                      return t & o;
                    })(t).toString(16),
                    e = (function (t) {
                      return (t & o) + r - 1;
                    })(t).toString(16);
                  return 'codepoint-index/plane' + (t >> 16) + '/' + n + '-' + e + '.json';
                }
                function i(t, n) {
                  var r = t & e,
                    o = n.codePointAt((r / 6) | 0);
                  return !!((o = (o || 48) - 48) & (1 << r % 6));
                }
                function c(t, n) {
                  !(function u(t, n) {
                    var r;
                    ((r = t),
                    r
                      .replace(/U\+/gi, '')
                      .replace(/^,+|,+$/g, '')
                      .split(/,+/)
                      .map(function (t) {
                        return t.split('-').map(function (t) {
                          return parseInt(t.trim(), 16);
                        });
                      })).forEach(function (t) {
                      var r = t[0],
                        e = t[1];
                      void 0 === e && (e = r), n(r, e);
                    });
                  })(t, function (t, r) {
                    for (var e = t; e <= r; e++) n(e);
                  });
                }
                var s = {},
                  f = {},
                  l = new WeakMap(),
                  v =
                    'https://cdn.jsdelivr.net/gh/lojjic/unicode-font-resolver@v1.0.1/packages/data';
                function d(t) {
                  var r = l.get(t);
                  return (
                    r ||
                      ((r = new n()),
                      c(t.ranges, function (t) {
                        return r.add(t);
                      }),
                      l.set(t, r)),
                    r
                  );
                }
                var h,
                  p = new Map();
                function g(t, n, r) {
                  return t[n]
                    ? n
                    : t[r]
                      ? r
                      : (function (t) {
                          for (var n in t) return n;
                        })(t);
                }
                function w(t, n) {
                  var r = n;
                  if (!t.includes(r)) {
                    r = 1 / 0;
                    for (var e = 0; e < t.length; e++)
                      Math.abs(t[e] - n) < Math.abs(r - n) && (r = t[e]);
                  }
                  return r;
                }
                function k(t) {
                  return (
                    h ||
                      ((h = new Set()),
                      c('9-D,20,85,A0,1680,2000-200A,2028-202F,205F,3000', function (t) {
                        h.add(t);
                      })),
                    h.has(t)
                  );
                }
                return (
                  (t.CodePointSet = n),
                  (t.clearCache = function () {
                    (s = {}), (f = {});
                  }),
                  (t.getFontsForString = function (t, n) {
                    void 0 === n && (n = {});
                    var r,
                      e = n.lang;
                    void 0 === e &&
                      (e = /\p{Script=Hangul}/u.test((r = t))
                        ? 'ko'
                        : /\p{Script=Hiragana}|\p{Script=Katakana}/u.test(r)
                          ? 'ja'
                          : 'en');
                    var o = n.category;
                    void 0 === o && (o = 'sans-serif');
                    var u = n.style;
                    void 0 === u && (u = 'normal');
                    var c = n.weight;
                    void 0 === c && (c = 400);
                    var l = (n.dataUrl || v).replace(/\/$/g, ''),
                      h = new Map(),
                      y = new Uint8Array(t.length),
                      b = {},
                      m = {},
                      A = new Array(t.length),
                      S = new Map(),
                      j = !1;
                    function M(t) {
                      var n = p.get(t);
                      return (
                        n ||
                          ((n = fetch(l + '/' + t)
                            .then(function (t) {
                              if (!t.ok) throw new Error(t.statusText);
                              return t.json().then(function (t) {
                                if (!Array.isArray(t) || 1 !== t[0])
                                  throw new Error('Incorrect schema version; need 1, got ' + t[0]);
                                return t[1];
                              });
                            })
                            .catch(function (n) {
                              if (l !== v)
                                return (
                                  j ||
                                    (console.error(
                                      'unicode-font-resolver: Failed loading from dataUrl "' +
                                        l +
                                        '", trying default CDN. ' +
                                        n.message,
                                    ),
                                    (j = !0)),
                                  (l = v),
                                  p.delete(t),
                                  M(t)
                                );
                              throw n;
                            })),
                          p.set(t, n)),
                        n
                      );
                    }
                    for (
                      var P = function (n) {
                          var r = t.codePointAt(n),
                            e = a(r);
                          (A[n] = e),
                            s[e] ||
                              S.has(e) ||
                              S.set(
                                e,
                                M(e).then(function (t) {
                                  s[e] = t;
                                }),
                              ),
                            r > 65535 && (n++, (E = n));
                        },
                        E = 0;
                      E < t.length;
                      E++
                    )
                      P(E);
                    return Promise.all(S.values())
                      .then(function () {
                        S.clear();
                        for (
                          var n = function (n) {
                              var o = t.codePointAt(n),
                                a = null,
                                u = s[A[n]],
                                c = void 0;
                              for (var l in u) {
                                var v = m[l];
                                if (
                                  (void 0 === v && (v = m[l] = new RegExp(l).test(e || 'en')), v)
                                ) {
                                  for (var d in ((c = l), u[l]))
                                    if (i(o, u[l][d])) {
                                      a = d;
                                      break;
                                    }
                                  break;
                                }
                              }
                              if (!a)
                                t: for (var h in u)
                                  if (h !== c)
                                    for (var p in u[h])
                                      if (i(o, u[h][p])) {
                                        a = p;
                                        break t;
                                      }
                              a ||
                                (console.debug('No font coverage for U+' + o.toString(16)),
                                (a = 'latin')),
                                (A[n] = a),
                                f[a] ||
                                  S.has(a) ||
                                  S.set(
                                    a,
                                    M('font-meta/' + a + '.json').then(function (t) {
                                      f[a] = t;
                                    }),
                                  ),
                                o > 65535 && (n++, (r = n));
                            },
                            r = 0;
                          r < t.length;
                          r++
                        )
                          n(r);
                        return Promise.all(S.values());
                      })
                      .then(function () {
                        for (var n, r = null, e = 0; e < t.length; e++) {
                          var a = t.codePointAt(e);
                          if (r && (k(a) || d(r).has(a))) y[e] = y[e - 1];
                          else {
                            r = f[A[e]];
                            var i = b[r.id];
                            if (!i) {
                              var s = r.typeforms,
                                v = g(s, o, 'sans-serif'),
                                p = g(s[v], u, 'normal'),
                                m = w(null === (n = s[v]) || void 0 === n ? void 0 : n[p], c);
                              i = b[r.id] =
                                l + '/font-files/' + r.id + '/' + v + '.' + p + '.' + m + '.woff';
                            }
                            var S = h.get(i);
                            null == S && ((S = h.size), h.set(i, S)), (y[e] = S);
                          }
                          a > 65535 && (e++, (y[e] = y[e - 1]));
                        }
                        return { fontUrls: Array.from(h.keys()), chars: y };
                      });
                  }),
                  Object.defineProperty(t, '__esModule', { value: !0 }),
                  t
                );
              })({});
            },
          ],
          init: (createFontResolver, fontParser, unicodeFontResolverClientFactory) =>
            createFontResolver(fontParser, unicodeFontResolverClientFactory()),
        });
        const now = () => (self.performance || Date).now(),
          mainThreadGenerator = SDFGenerator();
        let warned;
        const queue = [];
        let timer = 0;
        function nextChunk() {
          const start = now();
          for (; queue.length && now() - start < 5; ) queue.shift()();
          timer = queue.length ? setTimeout(nextChunk, 0) : 0;
        }
        const generateSDF_GL = (...args) =>
            new Promise((resolve, reject) => {
              queue.push(() => {
                const start = now();
                try {
                  mainThreadGenerator.webgl.generateIntoCanvas(...args),
                    resolve({ timing: now() - start });
                } catch (err) {
                  reject(err);
                }
              }),
                timer || (timer = setTimeout(nextChunk, 0));
            }),
          threadCount = 4,
          idleTimeout = 2e3,
          threads = {};
        let callNum = 0;
        function generateSDF_JS_Worker(
          width,
          height,
          path,
          viewBox,
          distance,
          exponent,
          canvas,
          x,
          y,
          channel,
        ) {
          const workerId = 'TroikaTextSDFGenerator_JS_' + (callNum++ % threadCount);
          let thread = threads[workerId];
          return (
            thread ||
              (thread = threads[workerId] =
                {
                  workerModule: defineWorkerModule({
                    name: workerId,
                    workerId,
                    dependencies: [SDFGenerator, now],
                    init(_createSDFGenerator, now) {
                      const generate = _createSDFGenerator().javascript.generate;
                      return function (...args) {
                        const start = now();
                        return { textureData: generate(...args), timing: now() - start };
                      };
                    },
                    getTransferables: (result) => [result.textureData.buffer],
                  }),
                  requests: 0,
                  idleTimer: null,
                }),
            thread.requests++,
            clearTimeout(thread.idleTimer),
            thread
              .workerModule(width, height, path, viewBox, distance, exponent)
              .then(({ textureData, timing }) => {
                const start = now(),
                  imageData = new Uint8Array(4 * textureData.length);
                for (let i = 0; i < textureData.length; i++)
                  imageData[4 * i + channel] = textureData[i];
                return (
                  mainThreadGenerator.webglUtils.renderImageData(
                    canvas,
                    imageData,
                    x,
                    y,
                    width,
                    height,
                    1 << (3 - channel),
                  ),
                  (timing += now() - start),
                  0 == --thread.requests &&
                    (thread.idleTimer = setTimeout(() => {
                      !(function terminateWorker(workerId) {
                        registeredModules[workerId] &&
                          registeredModules[workerId].forEach(function (unregister) {
                            unregister();
                          }),
                          workers[workerId] &&
                            (workers[workerId].terminate(), delete workers[workerId]);
                      })(workerId);
                    }, idleTimeout)),
                  { timing }
                );
              })
          );
        }
        const resizeWebGLCanvasWithoutClearing =
            mainThreadGenerator.webglUtils.resizeWebGLCanvasWithoutClearing,
          CONFIG = {
            defaultFontURL: null,
            unicodeFontsURL: null,
            sdfGlyphSize: 64,
            sdfMargin: 1 / 16,
            sdfExponent: 9,
            textureWidth: 2048,
          },
          tempColor = new three_module.Q1f();
        let hasRequested = !1;
        function now$1() {
          return (self.performance || Date).now();
        }
        function configureTextBuilder(config) {
          hasRequested
            ? console.warn('configureTextBuilder called after first font request; will be ignored.')
            : troika_three_text_esm_assign(CONFIG, config);
        }
        const atlases = Object.create(null);
        function getTextRenderInfo(args, callback) {
          (hasRequested = !0), (args = troika_three_text_esm_assign({}, args));
          const totalStart = now$1(),
            { defaultFontURL } = CONFIG,
            fonts = [];
          if (
            (defaultFontURL && fonts.push({ label: 'default', src: toAbsoluteURL(defaultFontURL) }),
            args.font && fonts.push({ label: 'user', src: toAbsoluteURL(args.font) }),
            (args.font = fonts),
            (args.text = '' + args.text),
            (args.sdfGlyphSize = args.sdfGlyphSize || CONFIG.sdfGlyphSize),
            (args.unicodeFontsURL = args.unicodeFontsURL || CONFIG.unicodeFontsURL),
            null != args.colorRanges)
          ) {
            let colors = {};
            for (let key in args.colorRanges)
              if (args.colorRanges.hasOwnProperty(key)) {
                let val = args.colorRanges[key];
                'number' != typeof val && (val = tempColor.set(val).getHex()), (colors[key] = val);
              }
            args.colorRanges = colors;
          }
          Object.freeze(args);
          const { textureWidth, sdfExponent } = CONFIG,
            { sdfGlyphSize } = args,
            glyphsPerRow = (textureWidth / sdfGlyphSize) * 4;
          let atlas = atlases[sdfGlyphSize];
          if (!atlas) {
            const canvas = document.createElement('canvas');
            (canvas.width = textureWidth),
              (canvas.height = (256 * sdfGlyphSize) / glyphsPerRow),
              (atlas = atlases[sdfGlyphSize] =
                {
                  glyphCount: 0,
                  sdfGlyphSize,
                  sdfCanvas: canvas,
                  sdfTexture: new three_module.gPd(
                    canvas,
                    void 0,
                    void 0,
                    void 0,
                    three_module.k6q,
                    three_module.k6q,
                  ),
                  contextLost: !1,
                  glyphsByFont: new Map(),
                }),
              (atlas.sdfTexture.generateMipmaps = !1),
              (function initContextLossHandling(atlas) {
                const canvas = atlas.sdfCanvas;
                canvas.addEventListener('webglcontextlost', (event) => {
                  console.log('Context Lost', event),
                    event.preventDefault(),
                    (atlas.contextLost = !0);
                }),
                  canvas.addEventListener('webglcontextrestored', (event) => {
                    console.log('Context Restored', event), (atlas.contextLost = !1);
                    const promises = [];
                    atlas.glyphsByFont.forEach((glyphMap) => {
                      glyphMap.forEach((glyph) => {
                        promises.push(generateGlyphSDF(glyph, atlas, !0));
                      });
                    }),
                      Promise.all(promises).then(() => {
                        safariPre15Workaround(atlas), (atlas.sdfTexture.needsUpdate = !0);
                      });
                  });
              })(atlas);
          }
          const { sdfTexture, sdfCanvas } = atlas;
          typesetInWorker(args).then((result) => {
            const { glyphIds, glyphFontIndices, fontData, glyphPositions, fontSize, timings } =
                result,
              neededSDFs = [],
              glyphBounds = new Float32Array(4 * glyphIds.length);
            let boundsIdx = 0,
              positionsIdx = 0;
            const quadsStart = now$1(),
              fontGlyphMaps = fontData.map((font) => {
                let map = atlas.glyphsByFont.get(font.src);
                return map || atlas.glyphsByFont.set(font.src, (map = new Map())), map;
              });
            glyphIds.forEach((glyphId, i) => {
              const fontIndex = glyphFontIndices[i],
                { src: fontSrc, unitsPerEm } = fontData[fontIndex];
              let glyphInfo = fontGlyphMaps[fontIndex].get(glyphId);
              if (!glyphInfo) {
                const { path, pathBounds } = result.glyphData[fontSrc][glyphId],
                  fontUnitsMargin =
                    (Math.max(pathBounds[2] - pathBounds[0], pathBounds[3] - pathBounds[1]) /
                      sdfGlyphSize) *
                    (CONFIG.sdfMargin * sdfGlyphSize + 0.5),
                  atlasIndex = atlas.glyphCount++,
                  sdfViewBox = [
                    pathBounds[0] - fontUnitsMargin,
                    pathBounds[1] - fontUnitsMargin,
                    pathBounds[2] + fontUnitsMargin,
                    pathBounds[3] + fontUnitsMargin,
                  ];
                fontGlyphMaps[fontIndex].set(
                  glyphId,
                  (glyphInfo = { path, atlasIndex, sdfViewBox }),
                ),
                  neededSDFs.push(glyphInfo);
              }
              const { sdfViewBox } = glyphInfo,
                posX = glyphPositions[positionsIdx++],
                posY = glyphPositions[positionsIdx++],
                fontSizeMult = fontSize / unitsPerEm;
              (glyphBounds[boundsIdx++] = posX + sdfViewBox[0] * fontSizeMult),
                (glyphBounds[boundsIdx++] = posY + sdfViewBox[1] * fontSizeMult),
                (glyphBounds[boundsIdx++] = posX + sdfViewBox[2] * fontSizeMult),
                (glyphBounds[boundsIdx++] = posY + sdfViewBox[3] * fontSizeMult),
                (glyphIds[i] = glyphInfo.atlasIndex);
            }),
              (timings.quads = (timings.quads || 0) + (now$1() - quadsStart));
            const sdfStart = now$1();
            timings.sdf = {};
            const currentHeight = sdfCanvas.height,
              neededRows = Math.ceil(atlas.glyphCount / glyphsPerRow),
              neededHeight = Math.pow(2, Math.ceil(Math.log2(neededRows * sdfGlyphSize)));
            neededHeight > currentHeight &&
              (console.info(`Increasing SDF texture size ${currentHeight}->${neededHeight}`),
              resizeWebGLCanvasWithoutClearing(sdfCanvas, textureWidth, neededHeight),
              sdfTexture.dispose()),
              Promise.all(
                neededSDFs.map((glyphInfo) =>
                  generateGlyphSDF(glyphInfo, atlas, args.gpuAccelerateSDF).then(({ timing }) => {
                    timings.sdf[glyphInfo.atlasIndex] = timing;
                  }),
                ),
              ).then(() => {
                neededSDFs.length &&
                  !atlas.contextLost &&
                  (safariPre15Workaround(atlas), (sdfTexture.needsUpdate = !0)),
                  (timings.sdfTotal = now$1() - sdfStart),
                  (timings.total = now$1() - totalStart),
                  callback(
                    Object.freeze({
                      parameters: args,
                      sdfTexture,
                      sdfGlyphSize,
                      sdfExponent,
                      glyphBounds,
                      glyphAtlasIndices: glyphIds,
                      glyphColors: result.glyphColors,
                      caretPositions: result.caretPositions,
                      chunkedBounds: result.chunkedBounds,
                      ascender: result.ascender,
                      descender: result.descender,
                      lineHeight: result.lineHeight,
                      capHeight: result.capHeight,
                      xHeight: result.xHeight,
                      topBaseline: result.topBaseline,
                      blockBounds: result.blockBounds,
                      visibleBounds: result.visibleBounds,
                      timings: result.timings,
                    }),
                  );
              });
          }),
            Promise.resolve().then(() => {
              atlas.contextLost ||
                (function warmUpSDFCanvas(canvas) {
                  canvas._warm ||
                    (mainThreadGenerator.webgl.isSupported(canvas), (canvas._warm = !0));
                })(sdfCanvas);
            });
        }
        function generateGlyphSDF(
          { path, atlasIndex, sdfViewBox },
          { sdfGlyphSize, sdfCanvas, contextLost },
          useGPU,
        ) {
          if (contextLost) return Promise.resolve({ timing: -1 });
          const { textureWidth, sdfExponent } = CONFIG,
            maxDist = Math.max(sdfViewBox[2] - sdfViewBox[0], sdfViewBox[3] - sdfViewBox[1]),
            squareIndex = Math.floor(atlasIndex / 4);
          return (function generateSDF(
            width,
            height,
            path,
            viewBox,
            distance,
            exponent,
            canvas,
            x,
            y,
            channel,
            useWebGL = !0,
          ) {
            return useWebGL
              ? generateSDF_GL(
                  width,
                  height,
                  path,
                  viewBox,
                  distance,
                  exponent,
                  canvas,
                  x,
                  y,
                  channel,
                ).then(
                  null,
                  (err) => (
                    warned ||
                      (console.warn('WebGL SDF generation failed, falling back to JS', err),
                      (warned = !0)),
                    generateSDF_JS_Worker(
                      width,
                      height,
                      path,
                      viewBox,
                      distance,
                      exponent,
                      canvas,
                      x,
                      y,
                      channel,
                    )
                  ),
                )
              : generateSDF_JS_Worker(
                  width,
                  height,
                  path,
                  viewBox,
                  distance,
                  exponent,
                  canvas,
                  x,
                  y,
                  channel,
                );
          })(
            sdfGlyphSize,
            sdfGlyphSize,
            path,
            sdfViewBox,
            maxDist,
            sdfExponent,
            sdfCanvas,
            (squareIndex % (textureWidth / sdfGlyphSize)) * sdfGlyphSize,
            Math.floor(squareIndex / (textureWidth / sdfGlyphSize)) * sdfGlyphSize,
            atlasIndex % 4,
            useGPU,
          );
        }
        function preloadFont({ font, characters, sdfGlyphSize }, callback) {
          getTextRenderInfo(
            {
              font,
              sdfGlyphSize,
              text: Array.isArray(characters) ? characters.join('\n') : '' + characters,
            },
            callback,
          );
        }
        function troika_three_text_esm_assign(toObj, fromObj) {
          for (let key in fromObj) fromObj.hasOwnProperty(key) && (toObj[key] = fromObj[key]);
          return toObj;
        }
        let linkEl;
        function toAbsoluteURL(path) {
          return (
            linkEl || (linkEl = 'undefined' == typeof document ? {} : document.createElement('a')),
            (linkEl.href = path),
            linkEl.href
          );
        }
        function safariPre15Workaround(atlas) {
          if ('function' != typeof createImageBitmap) {
            console.info('Safari<15: applying SDF canvas workaround');
            const { sdfCanvas, sdfTexture } = atlas,
              { width, height } = sdfCanvas,
              gl = atlas.sdfCanvas.getContext('webgl');
            let pixels = sdfTexture.image.data;
            (pixels && pixels.length === width * height * 4) ||
              ((pixels = new Uint8Array(width * height * 4)),
              (sdfTexture.image = { width, height, data: pixels }),
              (sdfTexture.flipY = !1),
              (sdfTexture.isDataTexture = !0)),
              gl.readPixels(0, 0, width, height, gl.RGBA, gl.UNSIGNED_BYTE, pixels);
          }
        }
        const typesetterWorkerModule = defineWorkerModule({
            name: 'Typesetter',
            dependencies: [
              function createTypesetter(resolveFonts, bidi) {
                const INF = 1 / 0,
                  DEFAULT_IGNORABLE_CHARS =
                    /[\u00AD\u034F\u061C\u115F-\u1160\u17B4-\u17B5\u180B-\u180E\u200B-\u200F\u202A-\u202E\u2060-\u206F\u3164\uFE00-\uFE0F\uFEFF\uFFA0\uFFF0-\uFFF8]/,
                  lineBreakingWhiteSpace = '[^\\S\\u00A0]',
                  BREAK_AFTER_CHARS = new RegExp(
                    `${lineBreakingWhiteSpace}|[\\-\\u007C\\u00AD\\u2010\\u2012-\\u2014\\u2027\\u2056\\u2E17\\u2E40]`,
                  );
                function typeset(
                  {
                    text = '',
                    font,
                    lang,
                    sdfGlyphSize = 64,
                    fontSize = 400,
                    fontWeight = 1,
                    fontStyle = 'normal',
                    letterSpacing = 0,
                    lineHeight = 'normal',
                    maxWidth = INF,
                    direction,
                    textAlign = 'left',
                    textIndent = 0,
                    whiteSpace = 'normal',
                    overflowWrap = 'normal',
                    anchorX = 0,
                    anchorY = 0,
                    metricsOnly = !1,
                    unicodeFontsURL,
                    preResolvedFonts = null,
                    includeCaretPositions = !1,
                    chunkedBoundsSize = 8192,
                    colorRanges = null,
                  },
                  callback,
                ) {
                  const mainStart = now(),
                    timings = { fontLoad: 0, typesetting: 0 };
                  text.indexOf('\r') > -1 &&
                    (console.info('Typesetter: got text with \\r chars; normalizing to \\n'),
                    (text = text.replace(/\r\n/g, '\n').replace(/\r/g, '\n'))),
                    (fontSize = +fontSize),
                    (letterSpacing = +letterSpacing),
                    (maxWidth = +maxWidth),
                    (lineHeight = lineHeight || 'normal'),
                    (textIndent = +textIndent),
                    (function calculateFontRuns(
                      { text, lang, fonts, style, weight, preResolvedFonts, unicodeFontsURL },
                      onDone,
                    ) {
                      const onResolved = ({ chars, fonts: parsedFonts }) => {
                        let curRun, prevVal;
                        const runs = [];
                        for (let i = 0; i < chars.length; i++)
                          chars[i] !== prevVal
                            ? ((prevVal = chars[i]),
                              runs.push(
                                (curRun = { start: i, end: i, fontObj: parsedFonts[chars[i]] }),
                              ))
                            : (curRun.end = i);
                        onDone(runs);
                      };
                      preResolvedFonts
                        ? onResolved(preResolvedFonts)
                        : resolveFonts(text, onResolved, {
                            lang,
                            fonts,
                            style,
                            weight,
                            unicodeFontsURL,
                          });
                    })(
                      {
                        text,
                        lang,
                        style: fontStyle,
                        weight: fontWeight,
                        fonts: 'string' == typeof font ? [{ src: font }] : font,
                        unicodeFontsURL,
                        preResolvedFonts,
                      },
                      (runs) => {
                        timings.fontLoad = now() - mainStart;
                        const hasMaxWidth = isFinite(maxWidth);
                        let glyphIds = null,
                          glyphFontIndices = null,
                          glyphPositions = null,
                          glyphData = null,
                          glyphColors = null,
                          caretPositions = null,
                          visibleBounds = null,
                          chunkedBounds = null,
                          maxLineWidth = 0,
                          renderableGlyphCount = 0,
                          canWrap = 'nowrap' !== whiteSpace;
                        const metricsByFont = new Map(),
                          typesetStart = now();
                        let lineXOffset = textIndent,
                          prevRunEndX = 0,
                          currentLine = new TextLine();
                        const lines = [currentLine];
                        runs.forEach((run) => {
                          const { fontObj } = run,
                            { ascender, descender, unitsPerEm, lineGap, capHeight, xHeight } =
                              fontObj;
                          let fontData = metricsByFont.get(fontObj);
                          if (!fontData) {
                            const fontSizeMult = fontSize / unitsPerEm,
                              calcLineHeight =
                                'normal' === lineHeight
                                  ? (ascender - descender + lineGap) * fontSizeMult
                                  : lineHeight * fontSize,
                              halfLeading =
                                (calcLineHeight - (ascender - descender) * fontSizeMult) / 2,
                              caretHeight = Math.min(
                                calcLineHeight,
                                (ascender - descender) * fontSizeMult,
                              ),
                              caretTop =
                                ((ascender + descender) / 2) * fontSizeMult + caretHeight / 2;
                            (fontData = {
                              index: metricsByFont.size,
                              src: fontObj.src,
                              fontObj,
                              fontSizeMult,
                              unitsPerEm,
                              ascender: ascender * fontSizeMult,
                              descender: descender * fontSizeMult,
                              capHeight: capHeight * fontSizeMult,
                              xHeight: xHeight * fontSizeMult,
                              lineHeight: calcLineHeight,
                              baseline: -halfLeading - ascender * fontSizeMult,
                              caretTop:
                                ((ascender + descender) / 2) * fontSizeMult + caretHeight / 2,
                              caretBottom: caretTop - caretHeight,
                            }),
                              metricsByFont.set(fontObj, fontData);
                          }
                          const { fontSizeMult } = fontData,
                            runText = text.slice(run.start, run.end + 1);
                          let prevGlyphX, prevGlyphObj;
                          fontObj.forEachGlyph(
                            runText,
                            fontSize,
                            letterSpacing,
                            (glyphObj, glyphX, glyphY, charIndex) => {
                              (glyphX += prevRunEndX),
                                (charIndex += run.start),
                                (prevGlyphX = glyphX),
                                (prevGlyphObj = glyphObj);
                              const char = text.charAt(charIndex),
                                glyphWidth = glyphObj.advanceWidth * fontSizeMult,
                                curLineCount = currentLine.count;
                              let nextLine;
                              if (
                                ('isEmpty' in glyphObj ||
                                  ((glyphObj.isWhitespace =
                                    !!char && new RegExp(lineBreakingWhiteSpace).test(char)),
                                  (glyphObj.canBreakAfter = !!char && BREAK_AFTER_CHARS.test(char)),
                                  (glyphObj.isEmpty =
                                    glyphObj.xMin === glyphObj.xMax ||
                                    glyphObj.yMin === glyphObj.yMax ||
                                    DEFAULT_IGNORABLE_CHARS.test(char))),
                                glyphObj.isWhitespace || glyphObj.isEmpty || renderableGlyphCount++,
                                canWrap &&
                                  hasMaxWidth &&
                                  !glyphObj.isWhitespace &&
                                  glyphX + glyphWidth + lineXOffset > maxWidth &&
                                  curLineCount)
                              ) {
                                if (currentLine.glyphAt(curLineCount - 1).glyphObj.canBreakAfter)
                                  (nextLine = new TextLine()), (lineXOffset = -glyphX);
                                else
                                  for (let i = curLineCount; i--; ) {
                                    if (0 === i && 'break-word' === overflowWrap) {
                                      (nextLine = new TextLine()), (lineXOffset = -glyphX);
                                      break;
                                    }
                                    if (currentLine.glyphAt(i).glyphObj.canBreakAfter) {
                                      nextLine = currentLine.splitAt(i + 1);
                                      const adjustX = nextLine.glyphAt(0).x;
                                      lineXOffset -= adjustX;
                                      for (let j = nextLine.count; j--; )
                                        nextLine.glyphAt(j).x -= adjustX;
                                      break;
                                    }
                                  }
                                nextLine &&
                                  ((currentLine.isSoftWrapped = !0),
                                  (currentLine = nextLine),
                                  lines.push(currentLine),
                                  (maxLineWidth = maxWidth));
                              }
                              let fly = currentLine.glyphAt(currentLine.count);
                              (fly.glyphObj = glyphObj),
                                (fly.x = glyphX + lineXOffset),
                                (fly.y = glyphY),
                                (fly.width = glyphWidth),
                                (fly.charIndex = charIndex),
                                (fly.fontData = fontData),
                                '\n' === char &&
                                  ((currentLine = new TextLine()),
                                  lines.push(currentLine),
                                  (lineXOffset =
                                    -(glyphX + glyphWidth + letterSpacing * fontSize) +
                                    textIndent));
                            },
                          ),
                            (prevRunEndX =
                              prevGlyphX +
                              prevGlyphObj.advanceWidth * fontSizeMult +
                              letterSpacing * fontSize);
                        });
                        let totalHeight = 0;
                        lines.forEach((line) => {
                          let isTrailingWhitespace = !0;
                          for (let i = line.count; i--; ) {
                            const glyphInfo = line.glyphAt(i);
                            isTrailingWhitespace &&
                              !glyphInfo.glyphObj.isWhitespace &&
                              ((line.width = glyphInfo.x + glyphInfo.width),
                              line.width > maxLineWidth && (maxLineWidth = line.width),
                              (isTrailingWhitespace = !1));
                            let { lineHeight, capHeight, xHeight, baseline } = glyphInfo.fontData;
                            lineHeight > line.lineHeight && (line.lineHeight = lineHeight);
                            const baselineDiff = baseline - line.baseline;
                            baselineDiff < 0 &&
                              ((line.baseline += baselineDiff),
                              (line.cap += baselineDiff),
                              (line.ex += baselineDiff)),
                              (line.cap = Math.max(line.cap, line.baseline + capHeight)),
                              (line.ex = Math.max(line.ex, line.baseline + xHeight));
                          }
                          (line.baseline -= totalHeight),
                            (line.cap -= totalHeight),
                            (line.ex -= totalHeight),
                            (totalHeight += line.lineHeight);
                        });
                        let anchorXOffset = 0,
                          anchorYOffset = 0;
                        if (
                          (anchorX &&
                            ('number' == typeof anchorX
                              ? (anchorXOffset = -anchorX)
                              : 'string' == typeof anchorX &&
                                (anchorXOffset =
                                  -maxLineWidth *
                                  ('left' === anchorX
                                    ? 0
                                    : 'center' === anchorX
                                      ? 0.5
                                      : 'right' === anchorX
                                        ? 1
                                        : parsePercent(anchorX)))),
                          anchorY &&
                            ('number' == typeof anchorY
                              ? (anchorYOffset = -anchorY)
                              : 'string' == typeof anchorY &&
                                (anchorYOffset =
                                  'top' === anchorY
                                    ? 0
                                    : 'top-baseline' === anchorY
                                      ? -lines[0].baseline
                                      : 'top-cap' === anchorY
                                        ? -lines[0].cap
                                        : 'top-ex' === anchorY
                                          ? -lines[0].ex
                                          : 'middle' === anchorY
                                            ? totalHeight / 2
                                            : 'bottom' === anchorY
                                              ? totalHeight
                                              : 'bottom-baseline' === anchorY
                                                ? lines[lines.length - 1].baseline
                                                : parsePercent(anchorY) * totalHeight)),
                          !metricsOnly)
                        ) {
                          const bidiLevelsResult = bidi.getEmbeddingLevels(text, direction);
                          (glyphIds = new Uint16Array(renderableGlyphCount)),
                            (glyphFontIndices = new Uint8Array(renderableGlyphCount)),
                            (glyphPositions = new Float32Array(2 * renderableGlyphCount)),
                            (glyphData = {}),
                            (visibleBounds = [INF, INF, -INF, -INF]),
                            (chunkedBounds = []),
                            includeCaretPositions &&
                              (caretPositions = new Float32Array(4 * text.length)),
                            colorRanges && (glyphColors = new Uint8Array(3 * renderableGlyphCount));
                          let chunk,
                            currentColor,
                            renderableGlyphIndex = 0,
                            prevCharIndex = -1,
                            colorCharIndex = -1;
                          if (
                            (lines.forEach((line, lineIndex) => {
                              let { count: lineGlyphCount, width: lineWidth } = line;
                              if (lineGlyphCount > 0) {
                                let trailingWhitespaceCount = 0;
                                for (
                                  let i = lineGlyphCount;
                                  i-- && line.glyphAt(i).glyphObj.isWhitespace;

                                )
                                  trailingWhitespaceCount++;
                                let lineXOffset = 0,
                                  justifyAdjust = 0;
                                if ('center' === textAlign)
                                  lineXOffset = (maxLineWidth - lineWidth) / 2;
                                else if ('right' === textAlign)
                                  lineXOffset = maxLineWidth - lineWidth;
                                else if ('justify' === textAlign && line.isSoftWrapped) {
                                  let whitespaceCount = 0;
                                  for (let i = lineGlyphCount - trailingWhitespaceCount; i--; )
                                    line.glyphAt(i).glyphObj.isWhitespace && whitespaceCount++;
                                  justifyAdjust = (maxLineWidth - lineWidth) / whitespaceCount;
                                }
                                if (justifyAdjust || lineXOffset) {
                                  let justifyOffset = 0;
                                  for (let i = 0; i < lineGlyphCount; i++) {
                                    let glyphInfo = line.glyphAt(i);
                                    const glyphObj = glyphInfo.glyphObj;
                                    (glyphInfo.x += lineXOffset + justifyOffset),
                                      0 !== justifyAdjust &&
                                        glyphObj.isWhitespace &&
                                        i < lineGlyphCount - trailingWhitespaceCount &&
                                        ((justifyOffset += justifyAdjust),
                                        (glyphInfo.width += justifyAdjust));
                                  }
                                }
                                const flips = bidi.getReorderSegments(
                                  text,
                                  bidiLevelsResult,
                                  line.glyphAt(0).charIndex,
                                  line.glyphAt(line.count - 1).charIndex,
                                );
                                for (let fi = 0; fi < flips.length; fi++) {
                                  const [start, end] = flips[fi];
                                  let left = 1 / 0,
                                    right = -1 / 0;
                                  for (let i = 0; i < lineGlyphCount; i++)
                                    if (line.glyphAt(i).charIndex >= start) {
                                      let startInLine = i,
                                        endInLine = i;
                                      for (; endInLine < lineGlyphCount; endInLine++) {
                                        let info = line.glyphAt(endInLine);
                                        if (info.charIndex > end) break;
                                        endInLine < lineGlyphCount - trailingWhitespaceCount &&
                                          ((left = Math.min(left, info.x)),
                                          (right = Math.max(right, info.x + info.width)));
                                      }
                                      for (let j = startInLine; j < endInLine; j++) {
                                        const glyphInfo = line.glyphAt(j);
                                        glyphInfo.x =
                                          right - (glyphInfo.x + glyphInfo.width - left);
                                      }
                                      break;
                                    }
                                }
                                let glyphObj;
                                const setGlyphObj = (g) => (glyphObj = g);
                                for (let i = 0; i < lineGlyphCount; i++) {
                                  const glyphInfo = line.glyphAt(i);
                                  glyphObj = glyphInfo.glyphObj;
                                  const glyphId = glyphObj.index,
                                    rtl = 1 & bidiLevelsResult.levels[glyphInfo.charIndex];
                                  if (rtl) {
                                    const mirrored = bidi.getMirroredCharacter(
                                      text[glyphInfo.charIndex],
                                    );
                                    mirrored &&
                                      glyphInfo.fontData.fontObj.forEachGlyph(
                                        mirrored,
                                        0,
                                        0,
                                        setGlyphObj,
                                      );
                                  }
                                  if (includeCaretPositions) {
                                    const { charIndex, fontData } = glyphInfo,
                                      caretLeft = glyphInfo.x + anchorXOffset,
                                      caretRight = glyphInfo.x + glyphInfo.width + anchorXOffset;
                                    (caretPositions[4 * charIndex] = rtl ? caretRight : caretLeft),
                                      (caretPositions[4 * charIndex + 1] = rtl
                                        ? caretLeft
                                        : caretRight),
                                      (caretPositions[4 * charIndex + 2] =
                                        line.baseline + fontData.caretBottom + anchorYOffset),
                                      (caretPositions[4 * charIndex + 3] =
                                        line.baseline + fontData.caretTop + anchorYOffset);
                                    const ligCount = charIndex - prevCharIndex;
                                    ligCount > 1 &&
                                      fillLigatureCaretPositions(
                                        caretPositions,
                                        prevCharIndex,
                                        ligCount,
                                      ),
                                      (prevCharIndex = charIndex);
                                  }
                                  if (colorRanges) {
                                    const { charIndex } = glyphInfo;
                                    for (; charIndex > colorCharIndex; )
                                      colorCharIndex++,
                                        colorRanges.hasOwnProperty(colorCharIndex) &&
                                          (currentColor = colorRanges[colorCharIndex]);
                                  }
                                  if (!glyphObj.isWhitespace && !glyphObj.isEmpty) {
                                    const idx = renderableGlyphIndex++,
                                      {
                                        fontSizeMult,
                                        src: fontSrc,
                                        index: fontIndex,
                                      } = glyphInfo.fontData,
                                      fontGlyphData =
                                        glyphData[fontSrc] || (glyphData[fontSrc] = {});
                                    fontGlyphData[glyphId] ||
                                      (fontGlyphData[glyphId] = {
                                        path: glyphObj.path,
                                        pathBounds: [
                                          glyphObj.xMin,
                                          glyphObj.yMin,
                                          glyphObj.xMax,
                                          glyphObj.yMax,
                                        ],
                                      });
                                    const glyphX = glyphInfo.x + anchorXOffset,
                                      glyphY = glyphInfo.y + line.baseline + anchorYOffset;
                                    (glyphPositions[2 * idx] = glyphX),
                                      (glyphPositions[2 * idx + 1] = glyphY);
                                    const visX0 = glyphX + glyphObj.xMin * fontSizeMult,
                                      visY0 = glyphY + glyphObj.yMin * fontSizeMult,
                                      visX1 = glyphX + glyphObj.xMax * fontSizeMult,
                                      visY1 = glyphY + glyphObj.yMax * fontSizeMult;
                                    visX0 < visibleBounds[0] && (visibleBounds[0] = visX0),
                                      visY0 < visibleBounds[1] && (visibleBounds[1] = visY0),
                                      visX1 > visibleBounds[2] && (visibleBounds[2] = visX1),
                                      visY1 > visibleBounds[3] && (visibleBounds[3] = visY1),
                                      idx % chunkedBoundsSize == 0 &&
                                        ((chunk = {
                                          start: idx,
                                          end: idx,
                                          rect: [INF, INF, -INF, -INF],
                                        }),
                                        chunkedBounds.push(chunk)),
                                      chunk.end++;
                                    const chunkRect = chunk.rect;
                                    if (
                                      (visX0 < chunkRect[0] && (chunkRect[0] = visX0),
                                      visY0 < chunkRect[1] && (chunkRect[1] = visY0),
                                      visX1 > chunkRect[2] && (chunkRect[2] = visX1),
                                      visY1 > chunkRect[3] && (chunkRect[3] = visY1),
                                      (glyphIds[idx] = glyphId),
                                      (glyphFontIndices[idx] = fontIndex),
                                      colorRanges)
                                    ) {
                                      const start = 3 * idx;
                                      (glyphColors[start] = (currentColor >> 16) & 255),
                                        (glyphColors[start + 1] = (currentColor >> 8) & 255),
                                        (glyphColors[start + 2] = 255 & currentColor);
                                    }
                                  }
                                }
                              }
                            }),
                            caretPositions)
                          ) {
                            const ligCount = text.length - prevCharIndex;
                            ligCount > 1 &&
                              fillLigatureCaretPositions(caretPositions, prevCharIndex, ligCount);
                          }
                        }
                        const fontData = [];
                        metricsByFont.forEach(
                          ({
                            index,
                            src,
                            unitsPerEm,
                            ascender,
                            descender,
                            lineHeight,
                            capHeight,
                            xHeight,
                          }) => {
                            fontData[index] = {
                              src,
                              unitsPerEm,
                              ascender,
                              descender,
                              lineHeight,
                              capHeight,
                              xHeight,
                            };
                          },
                        ),
                          (timings.typesetting = now() - typesetStart),
                          callback({
                            glyphIds,
                            glyphFontIndices,
                            glyphPositions,
                            glyphData,
                            fontData,
                            caretPositions,
                            glyphColors,
                            chunkedBounds,
                            fontSize,
                            topBaseline: anchorYOffset + lines[0].baseline,
                            blockBounds: [
                              anchorXOffset,
                              anchorYOffset - totalHeight,
                              anchorXOffset + maxLineWidth,
                              anchorYOffset,
                            ],
                            visibleBounds,
                            timings,
                          });
                      },
                    );
                }
                function parsePercent(str) {
                  let match = str.match(/^([\d.]+)%$/),
                    pct = match ? parseFloat(match[1]) : NaN;
                  return isNaN(pct) ? 0 : pct / 100;
                }
                function fillLigatureCaretPositions(caretPositions, ligStartIndex, ligCount) {
                  const ligStartX = caretPositions[4 * ligStartIndex],
                    ligEndX = caretPositions[4 * ligStartIndex + 1],
                    ligBottom = caretPositions[4 * ligStartIndex + 2],
                    ligTop = caretPositions[4 * ligStartIndex + 3],
                    guessedAdvanceX = (ligEndX - ligStartX) / ligCount;
                  for (let i = 0; i < ligCount; i++) {
                    const startIndex = 4 * (ligStartIndex + i);
                    (caretPositions[startIndex] = ligStartX + guessedAdvanceX * i),
                      (caretPositions[startIndex + 1] = ligStartX + guessedAdvanceX * (i + 1)),
                      (caretPositions[startIndex + 2] = ligBottom),
                      (caretPositions[startIndex + 3] = ligTop);
                  }
                }
                function now() {
                  return (self.performance || Date).now();
                }
                function TextLine() {
                  this.data = [];
                }
                const textLineProps = ['glyphObj', 'x', 'y', 'width', 'charIndex', 'fontData'];
                return (
                  (TextLine.prototype = {
                    width: 0,
                    lineHeight: 0,
                    baseline: 0,
                    cap: 0,
                    ex: 0,
                    isSoftWrapped: !1,
                    get count() {
                      return Math.ceil(this.data.length / textLineProps.length);
                    },
                    glyphAt(i) {
                      let fly = TextLine.flyweight;
                      return (fly.data = this.data), (fly.index = i), fly;
                    },
                    splitAt(i) {
                      let newLine = new TextLine();
                      return (newLine.data = this.data.splice(i * textLineProps.length)), newLine;
                    },
                  }),
                  (TextLine.flyweight = textLineProps.reduce(
                    (obj, prop, i, all) => (
                      Object.defineProperty(obj, prop, {
                        get() {
                          return this.data[this.index * textLineProps.length + i];
                        },
                        set(val) {
                          this.data[this.index * textLineProps.length + i] = val;
                        },
                      }),
                      obj
                    ),
                    { data: null, index: 0 },
                  )),
                  {
                    typeset,
                    measure: function measure(args, callback) {
                      typeset({ ...args, metricsOnly: !0 }, (result) => {
                        const [x0, y0, x1, y1] = result.blockBounds;
                        callback({ width: x1 - x0, height: y1 - y0 });
                      });
                    },
                  }
                );
              },
              fontResolverWorkerModule,
              bidi,
            ],
            init: (createTypesetter, fontResolver, bidiFactory) =>
              createTypesetter(fontResolver, bidiFactory()),
          }),
          typesetInWorker = defineWorkerModule({
            name: 'Typesetter',
            dependencies: [typesetterWorkerModule],
            init: (typesetter) =>
              function (args) {
                return new Promise((resolve) => {
                  typesetter.typeset(args, resolve);
                });
              },
            getTransferables(result) {
              const transferables = [];
              for (let p in result)
                result[p] && result[p].buffer && transferables.push(result[p].buffer);
              return transferables;
            },
          });
        function dumpSDFTextures() {
          Object.keys(atlases).forEach((size) => {
            const canvas = atlases[size].sdfCanvas,
              { width, height } = canvas;
            console.log(
              '%c.',
              `\n      background: url(${canvas.toDataURL()});\n      background-size: ${width}px ${height}px;\n      color: transparent;\n      font-size: 0;\n      line-height: ${height}px;\n      padding-left: ${width}px;\n    `,
            );
          });
        }
        const templateGeometries = {};
        class GlyphsGeometry extends three_module.CmU {
          constructor() {
            super(),
              (this.detail = 1),
              (this.curveRadius = 0),
              (this.groups = [
                { start: 0, count: 1 / 0, materialIndex: 0 },
                { start: 0, count: 1 / 0, materialIndex: 1 },
              ]),
              (this.boundingSphere = new three_module.iyt()),
              (this.boundingBox = new three_module.NRn());
          }
          computeBoundingSphere() {}
          computeBoundingBox() {}
          setSide(side) {
            const verts = this.getIndex().count;
            this.setDrawRange(
              side === three_module.hsX ? verts / 2 : 0,
              side === three_module.$EB ? verts : verts / 2,
            );
          }
          set detail(detail) {
            if (detail !== this._detail) {
              (this._detail = detail), ('number' != typeof detail || detail < 1) && (detail = 1);
              let tpl = (function getTemplateGeometry(detail) {
                let geom = templateGeometries[detail];
                if (!geom) {
                  const front = new three_module.bdM(1, 1, detail, detail),
                    back = front.clone(),
                    frontAttrs = front.attributes,
                    backAttrs = back.attributes,
                    combined = new three_module.LoY(),
                    vertCount = frontAttrs.uv.count;
                  for (let i = 0; i < vertCount; i++)
                    (backAttrs.position.array[3 * i] *= -1),
                      (backAttrs.normal.array[3 * i + 2] *= -1);
                  ['position', 'normal', 'uv'].forEach((name) => {
                    combined.setAttribute(
                      name,
                      new three_module.qtW(
                        [...frontAttrs[name].array, ...backAttrs[name].array],
                        frontAttrs[name].itemSize,
                      ),
                    );
                  }),
                    combined.setIndex([
                      ...front.index.array,
                      ...back.index.array.map((n) => n + vertCount),
                    ]),
                    combined.translate(0.5, 0.5, 0),
                    (geom = templateGeometries[detail] = combined);
                }
                return geom;
              })(detail);
              ['position', 'normal', 'uv'].forEach((attr) => {
                this.attributes[attr] = tpl.attributes[attr].clone();
              }),
                this.setIndex(tpl.getIndex().clone());
            }
          }
          get detail() {
            return this._detail;
          }
          set curveRadius(r) {
            r !== this._curveRadius && ((this._curveRadius = r), this._updateBounds());
          }
          get curveRadius() {
            return this._curveRadius;
          }
          updateGlyphs(glyphBounds, glyphAtlasIndices, blockBounds, chunkedBounds, glyphColors) {
            updateBufferAttr(this, 'aTroikaGlyphBounds', glyphBounds, 4),
              updateBufferAttr(this, 'aTroikaGlyphIndex', glyphAtlasIndices, 1),
              updateBufferAttr(this, 'aTroikaGlyphColor', glyphColors, 3),
              (this._blockBounds = blockBounds),
              (this._chunkedBounds = chunkedBounds),
              (this.instanceCount = glyphAtlasIndices.length),
              this._updateBounds();
          }
          _updateBounds() {
            const bounds = this._blockBounds;
            if (bounds) {
              const { curveRadius, boundingBox: bbox } = this;
              if (curveRadius) {
                const { PI, floor, min, max, sin, cos } = Math,
                  halfPi = PI / 2,
                  twoPi = 2 * PI,
                  absR = Math.abs(curveRadius),
                  leftAngle = bounds[0] / absR,
                  rightAngle = bounds[2] / absR,
                  minX =
                    floor((leftAngle + halfPi) / twoPi) !== floor((rightAngle + halfPi) / twoPi)
                      ? -absR
                      : min(sin(leftAngle) * absR, sin(rightAngle) * absR),
                  maxX =
                    floor((leftAngle - halfPi) / twoPi) !== floor((rightAngle - halfPi) / twoPi)
                      ? absR
                      : max(sin(leftAngle) * absR, sin(rightAngle) * absR),
                  maxZ =
                    floor((leftAngle + PI) / twoPi) !== floor((rightAngle + PI) / twoPi)
                      ? 2 * absR
                      : max(absR - cos(leftAngle) * absR, absR - cos(rightAngle) * absR);
                bbox.min.set(minX, bounds[1], curveRadius < 0 ? -maxZ : 0),
                  bbox.max.set(maxX, bounds[3], curveRadius < 0 ? 0 : maxZ);
              } else bbox.min.set(bounds[0], bounds[1], 0), bbox.max.set(bounds[2], bounds[3], 0);
              bbox.getBoundingSphere(this.boundingSphere);
            }
          }
          applyClipRect(clipRect) {
            let count = this.getAttribute('aTroikaGlyphIndex').count,
              chunks = this._chunkedBounds;
            if (chunks)
              for (let i = chunks.length; i--; ) {
                count = chunks[i].end;
                let rect = chunks[i].rect;
                if (
                  rect[1] < clipRect.w &&
                  rect[3] > clipRect.y &&
                  rect[0] < clipRect.z &&
                  rect[2] > clipRect.x
                )
                  break;
              }
            this.instanceCount = count;
          }
        }
        function updateBufferAttr(geom, attrName, newArray, itemSize) {
          const attr = geom.getAttribute(attrName);
          newArray
            ? attr && attr.array.length === newArray.length
              ? (attr.array.set(newArray), (attr.needsUpdate = !0))
              : (geom.setAttribute(attrName, new three_module.uWO(newArray, itemSize)),
                delete geom._maxInstanceCount,
                geom.dispose())
            : attr && geom.deleteAttribute(attrName);
        }
        const VERTEX_DEFS =
            '\nuniform vec2 uTroikaSDFTextureSize;\nuniform float uTroikaSDFGlyphSize;\nuniform vec4 uTroikaTotalBounds;\nuniform vec4 uTroikaClipRect;\nuniform mat3 uTroikaOrient;\nuniform bool uTroikaUseGlyphColors;\nuniform float uTroikaDistanceOffset;\nuniform float uTroikaBlurRadius;\nuniform vec2 uTroikaPositionOffset;\nuniform float uTroikaCurveRadius;\nattribute vec4 aTroikaGlyphBounds;\nattribute float aTroikaGlyphIndex;\nattribute vec3 aTroikaGlyphColor;\nvarying vec2 vTroikaGlyphUV;\nvarying vec4 vTroikaTextureUVBounds;\nvarying float vTroikaTextureChannel;\nvarying vec3 vTroikaGlyphColor;\nvarying vec2 vTroikaGlyphDimensions;\n',
          VERTEX_TRANSFORM =
            '\nvec4 bounds = aTroikaGlyphBounds;\nbounds.xz += uTroikaPositionOffset.x;\nbounds.yw -= uTroikaPositionOffset.y;\n\nvec4 outlineBounds = vec4(\n  bounds.xy - uTroikaDistanceOffset - uTroikaBlurRadius,\n  bounds.zw + uTroikaDistanceOffset + uTroikaBlurRadius\n);\nvec4 clippedBounds = vec4(\n  clamp(outlineBounds.xy, uTroikaClipRect.xy, uTroikaClipRect.zw),\n  clamp(outlineBounds.zw, uTroikaClipRect.xy, uTroikaClipRect.zw)\n);\n\nvec2 clippedXY = (mix(clippedBounds.xy, clippedBounds.zw, position.xy) - bounds.xy) / (bounds.zw - bounds.xy);\n\nposition.xy = mix(bounds.xy, bounds.zw, clippedXY);\n\nuv = (position.xy - uTroikaTotalBounds.xy) / (uTroikaTotalBounds.zw - uTroikaTotalBounds.xy);\n\nfloat rad = uTroikaCurveRadius;\nif (rad != 0.0) {\n  float angle = position.x / rad;\n  position.xz = vec2(sin(angle) * rad, rad - cos(angle) * rad);\n  normal.xz = vec2(sin(angle), cos(angle));\n}\n  \nposition = uTroikaOrient * position;\nnormal = uTroikaOrient * normal;\n\nvTroikaGlyphUV = clippedXY.xy;\nvTroikaGlyphDimensions = vec2(bounds[2] - bounds[0], bounds[3] - bounds[1]);\n\n\nfloat txCols = uTroikaSDFTextureSize.x / uTroikaSDFGlyphSize;\nvec2 txUvPerSquare = uTroikaSDFGlyphSize / uTroikaSDFTextureSize;\nvec2 txStartUV = txUvPerSquare * vec2(\n  mod(floor(aTroikaGlyphIndex / 4.0), txCols),\n  floor(floor(aTroikaGlyphIndex / 4.0) / txCols)\n);\nvTroikaTextureUVBounds = vec4(txStartUV, vec2(txStartUV) + txUvPerSquare);\nvTroikaTextureChannel = mod(aTroikaGlyphIndex, 4.0);\n',
          FRAGMENT_DEFS =
            "\nuniform sampler2D uTroikaSDFTexture;\nuniform vec2 uTroikaSDFTextureSize;\nuniform float uTroikaSDFGlyphSize;\nuniform float uTroikaSDFExponent;\nuniform float uTroikaDistanceOffset;\nuniform float uTroikaFillOpacity;\nuniform float uTroikaOutlineOpacity;\nuniform float uTroikaBlurRadius;\nuniform vec3 uTroikaStrokeColor;\nuniform float uTroikaStrokeWidth;\nuniform float uTroikaStrokeOpacity;\nuniform bool uTroikaSDFDebug;\nvarying vec2 vTroikaGlyphUV;\nvarying vec4 vTroikaTextureUVBounds;\nvarying float vTroikaTextureChannel;\nvarying vec2 vTroikaGlyphDimensions;\n\nfloat troikaSdfValueToSignedDistance(float alpha) {\n  // Inverse of exponential encoding in webgl-sdf-generator\n  \n  float maxDimension = max(vTroikaGlyphDimensions.x, vTroikaGlyphDimensions.y);\n  float absDist = (1.0 - pow(2.0 * (alpha > 0.5 ? 1.0 - alpha : alpha), 1.0 / uTroikaSDFExponent)) * maxDimension;\n  float signedDist = absDist * (alpha > 0.5 ? -1.0 : 1.0);\n  return signedDist;\n}\n\nfloat troikaGlyphUvToSdfValue(vec2 glyphUV) {\n  vec2 textureUV = mix(vTroikaTextureUVBounds.xy, vTroikaTextureUVBounds.zw, glyphUV);\n  vec4 rgba = texture2D(uTroikaSDFTexture, textureUV);\n  float ch = floor(vTroikaTextureChannel + 0.5); //NOTE: can't use round() in WebGL1\n  return ch == 0.0 ? rgba.r : ch == 1.0 ? rgba.g : ch == 2.0 ? rgba.b : rgba.a;\n}\n\nfloat troikaGlyphUvToDistance(vec2 uv) {\n  return troikaSdfValueToSignedDistance(troikaGlyphUvToSdfValue(uv));\n}\n\nfloat troikaGetAADist() {\n  \n  #if defined(GL_OES_standard_derivatives) || __VERSION__ >= 300\n  return length(fwidth(vTroikaGlyphUV * vTroikaGlyphDimensions)) * 0.5;\n  #else\n  return vTroikaGlyphDimensions.x / 64.0;\n  #endif\n}\n\nfloat troikaGetFragDistValue() {\n  vec2 clampedGlyphUV = clamp(vTroikaGlyphUV, 0.5 / uTroikaSDFGlyphSize, 1.0 - 0.5 / uTroikaSDFGlyphSize);\n  float distance = troikaGlyphUvToDistance(clampedGlyphUV);\n \n  // Extrapolate distance when outside bounds:\n  distance += clampedGlyphUV == vTroikaGlyphUV ? 0.0 : \n    length((vTroikaGlyphUV - clampedGlyphUV) * vTroikaGlyphDimensions);\n\n  \n\n  return distance;\n}\n\nfloat troikaGetEdgeAlpha(float distance, float distanceOffset, float aaDist) {\n  #if defined(IS_DEPTH_MATERIAL) || defined(IS_DISTANCE_MATERIAL)\n  float alpha = step(-distanceOffset, -distance);\n  #else\n\n  float alpha = smoothstep(\n    distanceOffset + aaDist,\n    distanceOffset - aaDist,\n    distance\n  );\n  #endif\n\n  return alpha;\n}\n",
          FRAGMENT_TRANSFORM =
            '\nfloat aaDist = troikaGetAADist();\nfloat fragDistance = troikaGetFragDistValue();\nfloat edgeAlpha = uTroikaSDFDebug ?\n  troikaGlyphUvToSdfValue(vTroikaGlyphUV) :\n  troikaGetEdgeAlpha(fragDistance, uTroikaDistanceOffset, max(aaDist, uTroikaBlurRadius));\n\n#if !defined(IS_DEPTH_MATERIAL) && !defined(IS_DISTANCE_MATERIAL)\nvec4 fillRGBA = gl_FragColor;\nfillRGBA.a *= uTroikaFillOpacity;\nvec4 strokeRGBA = uTroikaStrokeWidth == 0.0 ? fillRGBA : vec4(uTroikaStrokeColor, uTroikaStrokeOpacity);\nif (fillRGBA.a == 0.0) fillRGBA.rgb = strokeRGBA.rgb;\ngl_FragColor = mix(fillRGBA, strokeRGBA, smoothstep(\n  -uTroikaStrokeWidth - aaDist,\n  -uTroikaStrokeWidth + aaDist,\n  fragDistance\n));\ngl_FragColor.a *= edgeAlpha;\n#endif\n\nif (edgeAlpha == 0.0) {\n  discard;\n}\n';
        function createTextDerivedMaterial(baseMaterial) {
          const textMaterial = createDerivedMaterial(baseMaterial, {
            chained: !0,
            extensions: { derivatives: !0 },
            uniforms: {
              uTroikaSDFTexture: { value: null },
              uTroikaSDFTextureSize: { value: new three_module.I9Y() },
              uTroikaSDFGlyphSize: { value: 0 },
              uTroikaSDFExponent: { value: 0 },
              uTroikaTotalBounds: { value: new three_module.IUQ(0, 0, 0, 0) },
              uTroikaClipRect: { value: new three_module.IUQ(0, 0, 0, 0) },
              uTroikaDistanceOffset: { value: 0 },
              uTroikaOutlineOpacity: { value: 0 },
              uTroikaFillOpacity: { value: 1 },
              uTroikaPositionOffset: { value: new three_module.I9Y() },
              uTroikaCurveRadius: { value: 0 },
              uTroikaBlurRadius: { value: 0 },
              uTroikaStrokeWidth: { value: 0 },
              uTroikaStrokeColor: { value: new three_module.Q1f() },
              uTroikaStrokeOpacity: { value: 1 },
              uTroikaOrient: { value: new three_module.dwI() },
              uTroikaUseGlyphColors: { value: !0 },
              uTroikaSDFDebug: { value: !1 },
            },
            vertexDefs: VERTEX_DEFS,
            vertexTransform: VERTEX_TRANSFORM,
            fragmentDefs: FRAGMENT_DEFS,
            fragmentColorTransform: FRAGMENT_TRANSFORM,
            customRewriter({ vertexShader, fragmentShader }) {
              let uDiffuseRE = /\buniform\s+vec3\s+diffuse\b/;
              return (
                uDiffuseRE.test(fragmentShader) &&
                  ((fragmentShader = fragmentShader
                    .replace(uDiffuseRE, 'varying vec3 vTroikaGlyphColor')
                    .replace(/\bdiffuse\b/g, 'vTroikaGlyphColor')),
                  uDiffuseRE.test(vertexShader) ||
                    (vertexShader = vertexShader.replace(
                      voidMainRegExp,
                      'uniform vec3 diffuse;\n$&\nvTroikaGlyphColor = uTroikaUseGlyphColors ? aTroikaGlyphColor / 255.0 : diffuse;\n',
                    ))),
                { vertexShader, fragmentShader }
              );
            },
          });
          return (
            (textMaterial.transparent = !0),
            Object.defineProperties(textMaterial, {
              isTroikaTextMaterial: { value: !0 },
              shadowSide: {
                get() {
                  return this.side;
                },
                set() {},
              },
            }),
            textMaterial
          );
        }
        const defaultMaterial = new three_module.V9B({
            color: 16777215,
            side: three_module.$EB,
            transparent: !0,
          }),
          tempMat4 = new three_module.kn4(),
          tempVec3a = new three_module.Pq0(),
          tempVec3b = new three_module.Pq0(),
          tempArray = [],
          origin = new three_module.Pq0();
        function first(o) {
          return Array.isArray(o) ? o[0] : o;
        }
        let getFlatRaycastMesh = () => {
            const mesh = new three_module.eaF(new three_module.bdM(1, 1), defaultMaterial);
            return (getFlatRaycastMesh = () => mesh), mesh;
          },
          getCurvedRaycastMesh = () => {
            const mesh = new three_module.eaF(new three_module.bdM(1, 1, 32, 1), defaultMaterial);
            return (getCurvedRaycastMesh = () => mesh), mesh;
          };
        const syncStartEvent = { type: 'syncstart' },
          syncCompleteEvent = { type: 'synccomplete' },
          SYNCABLE_PROPS = [
            'font',
            'fontSize',
            'fontStyle',
            'fontWeight',
            'lang',
            'letterSpacing',
            'lineHeight',
            'maxWidth',
            'overflowWrap',
            'text',
            'direction',
            'textAlign',
            'textIndent',
            'whiteSpace',
            'anchorX',
            'anchorY',
            'colorRanges',
            'sdfGlyphSize',
          ],
          COPYABLE_PROPS = SYNCABLE_PROPS.concat(
            'material',
            'color',
            'depthOffset',
            'clipRect',
            'curveRadius',
            'orientation',
            'glyphGeometryDetail',
          );
        class Text extends three_module.eaF {
          constructor() {
            super(new GlyphsGeometry(), null),
              (this.text = ''),
              (this.anchorX = 0),
              (this.anchorY = 0),
              (this.curveRadius = 0),
              (this.direction = 'auto'),
              (this.font = null),
              (this.unicodeFontsURL = null),
              (this.fontSize = 0.1),
              (this.fontWeight = 'normal'),
              (this.fontStyle = 'normal'),
              (this.lang = null),
              (this.letterSpacing = 0),
              (this.lineHeight = 'normal'),
              (this.maxWidth = 1 / 0),
              (this.overflowWrap = 'normal'),
              (this.textAlign = 'left'),
              (this.textIndent = 0),
              (this.whiteSpace = 'normal'),
              (this.material = null),
              (this.color = null),
              (this.colorRanges = null),
              (this.outlineWidth = 0),
              (this.outlineColor = 0),
              (this.outlineOpacity = 1),
              (this.outlineBlur = 0),
              (this.outlineOffsetX = 0),
              (this.outlineOffsetY = 0),
              (this.strokeWidth = 0),
              (this.strokeColor = 8421504),
              (this.strokeOpacity = 1),
              (this.fillOpacity = 1),
              (this.depthOffset = 0),
              (this.clipRect = null),
              (this.orientation = '+x+y'),
              (this.glyphGeometryDetail = 1),
              (this.sdfGlyphSize = null),
              (this.gpuAccelerateSDF = !0),
              (this.debugSDF = !1);
          }
          sync(callback) {
            this._needsSync &&
              ((this._needsSync = !1),
              this._isSyncing
                ? (this._queuedSyncs || (this._queuedSyncs = [])).push(callback)
                : ((this._isSyncing = !0),
                  this.dispatchEvent(syncStartEvent),
                  getTextRenderInfo(
                    {
                      text: this.text,
                      font: this.font,
                      lang: this.lang,
                      fontSize: this.fontSize || 0.1,
                      fontWeight: this.fontWeight || 'normal',
                      fontStyle: this.fontStyle || 'normal',
                      letterSpacing: this.letterSpacing || 0,
                      lineHeight: this.lineHeight || 'normal',
                      maxWidth: this.maxWidth,
                      direction: this.direction || 'auto',
                      textAlign: this.textAlign,
                      textIndent: this.textIndent,
                      whiteSpace: this.whiteSpace,
                      overflowWrap: this.overflowWrap,
                      anchorX: this.anchorX,
                      anchorY: this.anchorY,
                      colorRanges: this.colorRanges,
                      includeCaretPositions: !0,
                      sdfGlyphSize: this.sdfGlyphSize,
                      gpuAccelerateSDF: this.gpuAccelerateSDF,
                      unicodeFontsURL: this.unicodeFontsURL,
                    },
                    (textRenderInfo) => {
                      (this._isSyncing = !1),
                        (this._textRenderInfo = textRenderInfo),
                        this.geometry.updateGlyphs(
                          textRenderInfo.glyphBounds,
                          textRenderInfo.glyphAtlasIndices,
                          textRenderInfo.blockBounds,
                          textRenderInfo.chunkedBounds,
                          textRenderInfo.glyphColors,
                        );
                      const queued = this._queuedSyncs;
                      queued &&
                        ((this._queuedSyncs = null),
                        (this._needsSync = !0),
                        this.sync(() => {
                          queued.forEach((fn) => fn && fn());
                        })),
                        this.dispatchEvent(syncCompleteEvent),
                        callback && callback();
                    },
                  )));
          }
          onBeforeRender(renderer, scene, camera, geometry, material, group) {
            this.sync(),
              material.isTroikaTextMaterial && this._prepareForRender(material),
              (material._hadOwnSide = material.hasOwnProperty('side')),
              this.geometry.setSide((material._actualSide = material.side)),
              (material.side = three_module.hB5);
          }
          onAfterRender(renderer, scene, camera, geometry, material, group) {
            material._hadOwnSide ? (material.side = material._actualSide) : delete material.side;
          }
          dispose() {
            this.geometry.dispose();
          }
          get textRenderInfo() {
            return this._textRenderInfo || null;
          }
          get material() {
            let derivedMaterial = this._derivedMaterial;
            const baseMaterial =
              this._baseMaterial ||
              this._defaultMaterial ||
              (this._defaultMaterial = defaultMaterial.clone());
            if (
              ((derivedMaterial && derivedMaterial.baseMaterial === baseMaterial) ||
                ((derivedMaterial = this._derivedMaterial =
                  createTextDerivedMaterial(baseMaterial)),
                baseMaterial.addEventListener('dispose', function onDispose() {
                  baseMaterial.removeEventListener('dispose', onDispose), derivedMaterial.dispose();
                })),
              this.outlineWidth || this.outlineBlur || this.outlineOffsetX || this.outlineOffsetY)
            ) {
              let outlineMaterial = derivedMaterial._outlineMtl;
              return (
                outlineMaterial ||
                  ((outlineMaterial = derivedMaterial._outlineMtl =
                    Object.create(derivedMaterial, { id: { value: derivedMaterial.id + 0.1 } })),
                  (outlineMaterial.isTextOutlineMaterial = !0),
                  (outlineMaterial.depthWrite = !1),
                  (outlineMaterial.map = null),
                  derivedMaterial.addEventListener('dispose', function onDispose() {
                    derivedMaterial.removeEventListener('dispose', onDispose),
                      outlineMaterial.dispose();
                  })),
                [outlineMaterial, derivedMaterial]
              );
            }
            return derivedMaterial;
          }
          set material(baseMaterial) {
            baseMaterial && baseMaterial.isTroikaTextMaterial
              ? ((this._derivedMaterial = baseMaterial),
                (this._baseMaterial = baseMaterial.baseMaterial))
              : (this._baseMaterial = baseMaterial);
          }
          get glyphGeometryDetail() {
            return this.geometry.detail;
          }
          set glyphGeometryDetail(detail) {
            this.geometry.detail = detail;
          }
          get curveRadius() {
            return this.geometry.curveRadius;
          }
          set curveRadius(r) {
            this.geometry.curveRadius = r;
          }
          get customDepthMaterial() {
            return first(this.material).getDepthMaterial();
          }
          get customDistanceMaterial() {
            return first(this.material).getDistanceMaterial();
          }
          _prepareForRender(material) {
            const isOutline = material.isTextOutlineMaterial,
              uniforms = material.uniforms,
              textInfo = this.textRenderInfo;
            if (textInfo) {
              const { sdfTexture, blockBounds } = textInfo;
              (uniforms.uTroikaSDFTexture.value = sdfTexture),
                uniforms.uTroikaSDFTextureSize.value.set(
                  sdfTexture.image.width,
                  sdfTexture.image.height,
                ),
                (uniforms.uTroikaSDFGlyphSize.value = textInfo.sdfGlyphSize),
                (uniforms.uTroikaSDFExponent.value = textInfo.sdfExponent),
                uniforms.uTroikaTotalBounds.value.fromArray(blockBounds),
                (uniforms.uTroikaUseGlyphColors.value = !isOutline && !!textInfo.glyphColors);
              let fillOpacity,
                strokeOpacity,
                strokeColor,
                distanceOffset = 0,
                blurRadius = 0,
                strokeWidth = 0,
                offsetX = 0,
                offsetY = 0;
              if (isOutline) {
                let { outlineWidth, outlineOffsetX, outlineOffsetY, outlineBlur, outlineOpacity } =
                  this;
                (distanceOffset = this._parsePercent(outlineWidth) || 0),
                  (blurRadius = Math.max(0, this._parsePercent(outlineBlur) || 0)),
                  (fillOpacity = outlineOpacity),
                  (offsetX = this._parsePercent(outlineOffsetX) || 0),
                  (offsetY = this._parsePercent(outlineOffsetY) || 0);
              } else
                (strokeWidth = Math.max(0, this._parsePercent(this.strokeWidth) || 0)),
                  strokeWidth &&
                    ((strokeColor = this.strokeColor),
                    uniforms.uTroikaStrokeColor.value.set(
                      null == strokeColor ? 8421504 : strokeColor,
                    ),
                    (strokeOpacity = this.strokeOpacity),
                    null == strokeOpacity && (strokeOpacity = 1)),
                  (fillOpacity = this.fillOpacity);
              (uniforms.uTroikaDistanceOffset.value = distanceOffset),
                uniforms.uTroikaPositionOffset.value.set(offsetX, offsetY),
                (uniforms.uTroikaBlurRadius.value = blurRadius),
                (uniforms.uTroikaStrokeWidth.value = strokeWidth),
                (uniforms.uTroikaStrokeOpacity.value = strokeOpacity),
                (uniforms.uTroikaFillOpacity.value = null == fillOpacity ? 1 : fillOpacity),
                (uniforms.uTroikaCurveRadius.value = this.curveRadius || 0);
              let clipRect = this.clipRect;
              if (clipRect && Array.isArray(clipRect) && 4 === clipRect.length)
                uniforms.uTroikaClipRect.value.fromArray(clipRect);
              else {
                const pad = 100 * (this.fontSize || 0.1);
                uniforms.uTroikaClipRect.value.set(
                  blockBounds[0] - pad,
                  blockBounds[1] - pad,
                  blockBounds[2] + pad,
                  blockBounds[3] + pad,
                );
              }
              this.geometry.applyClipRect(uniforms.uTroikaClipRect.value);
            }
            (uniforms.uTroikaSDFDebug.value = !!this.debugSDF),
              (material.polygonOffset = !!this.depthOffset),
              (material.polygonOffsetFactor = material.polygonOffsetUnits = this.depthOffset || 0);
            const color = isOutline ? this.outlineColor || 0 : this.color;
            if (null == color) delete material.color;
            else {
              const colorObj = material.hasOwnProperty('color')
                ? material.color
                : (material.color = new three_module.Q1f());
              (color === colorObj._input && 'object' != typeof color) ||
                colorObj.set((colorObj._input = color));
            }
            let orient = this.orientation || '+x+y';
            if (orient !== material._orientation) {
              let rotMat = uniforms.uTroikaOrient.value;
              orient = orient.replace(/[^-+xyz]/g, '');
              let match = '+x+y' !== orient && orient.match(/^([-+])([xyz])([-+])([xyz])$/);
              if (match) {
                let [, hSign, hAxis, vSign, vAxis] = match;
                (tempVec3a.set(0, 0, 0)[hAxis] = '-' === hSign ? 1 : -1),
                  (tempVec3b.set(0, 0, 0)[vAxis] = '-' === vSign ? -1 : 1),
                  tempMat4.lookAt(origin, tempVec3a.cross(tempVec3b), tempVec3b),
                  rotMat.setFromMatrix4(tempMat4);
              } else rotMat.identity();
              material._orientation = orient;
            }
          }
          _parsePercent(value) {
            if ('string' == typeof value) {
              let match = value.match(/^(-?[\d.]+)%$/),
                pct = match ? parseFloat(match[1]) : NaN;
              value = (isNaN(pct) ? 0 : pct / 100) * this.fontSize;
            }
            return value;
          }
          localPositionToTextCoords(position, target = new three_module.I9Y()) {
            target.copy(position);
            const r = this.curveRadius;
            return (
              r &&
                (target.x =
                  Math.atan2(position.x, Math.abs(r) - Math.abs(position.z)) * Math.abs(r)),
              target
            );
          }
          worldPositionToTextCoords(position, target = new three_module.I9Y()) {
            return (
              tempVec3a.copy(position),
              this.localPositionToTextCoords(this.worldToLocal(tempVec3a), target)
            );
          }
          raycast(raycaster, intersects) {
            const { textRenderInfo, curveRadius } = this;
            if (textRenderInfo) {
              const bounds = textRenderInfo.blockBounds,
                raycastMesh = curveRadius ? getCurvedRaycastMesh() : getFlatRaycastMesh(),
                geom = raycastMesh.geometry,
                { position, uv } = geom.attributes;
              for (let i = 0; i < uv.count; i++) {
                let x = bounds[0] + uv.getX(i) * (bounds[2] - bounds[0]);
                const y = bounds[1] + uv.getY(i) * (bounds[3] - bounds[1]);
                let z = 0;
                curveRadius &&
                  ((z = curveRadius - Math.cos(x / curveRadius) * curveRadius),
                  (x = Math.sin(x / curveRadius) * curveRadius)),
                  position.setXYZ(i, x, y, z);
              }
              (geom.boundingSphere = this.geometry.boundingSphere),
                (geom.boundingBox = this.geometry.boundingBox),
                (raycastMesh.matrixWorld = this.matrixWorld),
                (raycastMesh.material.side = this.material.side),
                (tempArray.length = 0),
                raycastMesh.raycast(raycaster, tempArray);
              for (let i = 0; i < tempArray.length; i++)
                (tempArray[i].object = this), intersects.push(tempArray[i]);
            }
          }
          copy(source) {
            const geom = this.geometry;
            return (
              super.copy(source),
              (this.geometry = geom),
              COPYABLE_PROPS.forEach((prop) => {
                this[prop] = source[prop];
              }),
              this
            );
          }
          clone() {
            return new this.constructor().copy(this);
          }
        }
        function getCaretAtPoint(textRenderInfo, x, y) {
          let closestCaret = null;
          const rows = (function groupCaretsByRow(textRenderInfo) {
            let rows = _caretsByRowCache.get(textRenderInfo);
            if (!rows) {
              rows = [];
              const { caretPositions } = textRenderInfo;
              let curRow;
              const visitCaret = (x, bottom, top, charIndex) => {
                (!curRow || top < (curRow.top + curRow.bottom) / 2) &&
                  rows.push((curRow = { bottom, top, carets: [] })),
                  top > curRow.top && (curRow.top = top),
                  bottom < curRow.bottom && (curRow.bottom = bottom),
                  curRow.carets.push({ x, y: bottom, height: top - bottom, charIndex });
              };
              let i = 0;
              for (; i < caretPositions.length; i += 4)
                visitCaret(caretPositions[i], caretPositions[i + 2], caretPositions[i + 3], i / 4);
              visitCaret(
                caretPositions[i - 3],
                caretPositions[i - 2],
                caretPositions[i - 1],
                i / 4,
              );
            }
            return _caretsByRowCache.set(textRenderInfo, rows), rows;
          })(textRenderInfo);
          let closestRow = null;
          return (
            rows.forEach((row) => {
              (!closestRow ||
                Math.abs(y - (row.top + row.bottom) / 2) <
                  Math.abs(y - (closestRow.top + closestRow.bottom) / 2)) &&
                (closestRow = row);
            }),
            closestRow.carets.forEach((caret) => {
              (!closestCaret || Math.abs(x - caret.x) < Math.abs(x - closestCaret.x)) &&
                (closestCaret = caret);
            }),
            closestCaret
          );
        }
        SYNCABLE_PROPS.forEach((prop) => {
          const privateKey = '_private_' + prop;
          Object.defineProperty(Text.prototype, prop, {
            get() {
              return this[privateKey];
            },
            set(value) {
              value !== this[privateKey] && ((this[privateKey] = value), (this._needsSync = !0));
            },
          });
        });
        const _rectsCache = new WeakMap();
        function getSelectionRects(textRenderInfo, start, end) {
          let rects;
          if (textRenderInfo) {
            let prevResult = _rectsCache.get(textRenderInfo);
            if (prevResult && prevResult.start === start && prevResult.end === end)
              return prevResult.rects;
            const { caretPositions } = textRenderInfo;
            if (end < start) {
              const s = start;
              (start = end), (end = s);
            }
            (start = Math.max(start, 0)),
              (end = Math.min(end, caretPositions.length + 1)),
              (rects = []);
            let currentRect = null;
            for (let i = start; i < end; i++) {
              const x1 = caretPositions[4 * i],
                x2 = caretPositions[4 * i + 1],
                left = Math.min(x1, x2),
                right = Math.max(x1, x2),
                bottom = caretPositions[4 * i + 2],
                top = caretPositions[4 * i + 3];
              (!currentRect ||
                bottom !== currentRect.bottom ||
                top !== currentRect.top ||
                left > currentRect.right ||
                right < currentRect.left) &&
                ((currentRect = { left: 1 / 0, right: -1 / 0, bottom, top }),
                rects.push(currentRect)),
                (currentRect.left = Math.min(left, currentRect.left)),
                (currentRect.right = Math.max(right, currentRect.right));
            }
            rects.sort((a, b) => b.bottom - a.bottom || a.left - b.left);
            for (let i = rects.length - 1; i-- > 0; ) {
              const rectA = rects[i],
                rectB = rects[i + 1];
              rectA.bottom === rectB.bottom &&
                rectA.top === rectB.top &&
                rectA.left <= rectB.right &&
                rectA.right >= rectB.left &&
                ((rectB.left = Math.min(rectB.left, rectA.left)),
                (rectB.right = Math.max(rectB.right, rectA.right)),
                rects.splice(i, 1));
            }
            _rectsCache.set(textRenderInfo, { start, end, rects });
          }
          return rects;
        }
        const _caretsByRowCache = new WeakMap();
      },
  },
]);
