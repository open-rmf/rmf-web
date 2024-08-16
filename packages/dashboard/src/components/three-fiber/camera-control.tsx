import { useFrame, useThree } from '@react-three/fiber';
import React, { useEffect, useRef } from 'react';
import { Subscription } from 'rxjs';
import { MOUSE, Vector3 } from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';

import { AppEvents } from '../app-events';

const DEFAULT_ZOOM_IN_CONSTANT = 1.2;
const DEFAULT_ZOOM_OUT_CONSTANT = 0.8;

interface CameraControlProps {
  zoom: number;
}

export const CameraControl: React.FC<CameraControlProps> = ({ zoom }) => {
  const { camera, gl } = useThree();
  const controlsRef = useRef<OrbitControls | null>(null);

  useEffect(() => {
    const controls = new OrbitControls(camera, gl.domElement);
    controls.target = new Vector3(0, 0, -Number.MAX_SAFE_INTEGER);
    controls.enableRotate = false;
    controls.enableDamping = false;
    controls.enableZoom = true;
    controls.mouseButtons = {
      LEFT: MOUSE.PAN,
      MIDDLE: undefined,
      RIGHT: undefined,
    };
    controlsRef.current = controls;

    const subs: Subscription[] = [];
    subs.push(
      AppEvents.zoomIn.subscribe(() => {
        const newZoom = camera.zoom * DEFAULT_ZOOM_IN_CONSTANT;
        camera.zoom = newZoom;
        AppEvents.zoom.next(newZoom);
        camera.updateProjectionMatrix();
      }),
    );
    subs.push(
      AppEvents.zoomOut.subscribe(() => {
        const newZoom = camera.zoom * DEFAULT_ZOOM_OUT_CONSTANT;
        camera.zoom = newZoom;
        AppEvents.zoom.next(newZoom);
        camera.updateProjectionMatrix();
      }),
    );
    subs.push(
      AppEvents.resetCamera.subscribe((data) => {
        camera.position.set(data[0], data[1], data[2]);
        camera.zoom = data[3];
        AppEvents.zoom.next(data[3]);
        camera.updateProjectionMatrix();
      }),
    );

    const handleScroll = (event: WheelEvent) => {
      const SENSITIVITY = 0.9;
      /**
       * event.deltaY represents the vertical scroll amount generated by the mouse wheel,
       * with positive values indicating downward scroll (toward the user)
       * and negative values indicating upward scroll (away from the user).
       */
      const newZoom =
        event.deltaY > 0
          ? camera.zoom * DEFAULT_ZOOM_OUT_CONSTANT
          : camera.zoom * DEFAULT_ZOOM_IN_CONSTANT;
      AppEvents.zoom.next(newZoom * SENSITIVITY);
    };

    gl.domElement.addEventListener('wheel', handleScroll);

    return () => {
      for (const sub of subs) {
        sub.unsubscribe();
      }
      gl.domElement.removeEventListener('wheel', handleScroll);
      controls.dispose();
    };
  }, [camera, gl.domElement]);

  useEffect(() => {
    camera.zoom = zoom;
    if (AppEvents.cameraPosition.value) {
      camera.position.set(
        AppEvents.cameraPosition.value.x,
        AppEvents.cameraPosition.value.y,
        AppEvents.cameraPosition.value.z,
      );
    }
  }, [camera, zoom]);

  useFrame(() => {
    if (controlsRef.current) {
      controlsRef.current.update();
      AppEvents.cameraPosition.next(new Vector3().copy(camera.position));
    }
  });

  return null;
};
