import { AppEvents } from '../app-events';
import { Subscription } from 'rxjs';
import { useFrame, useThree } from '@react-three/fiber';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls';
import React, { useEffect, useRef } from 'react';
import { MOUSE, Vector3 } from 'three';

const DEFAULT_ZOOM_IN_CONSTANT = 1.2;
const DEFAULT_ZOOM_OUT_CONSTANT = 0.8;

interface CameraControlProps {
  zoom: number;
}

export const CameraControl: React.FC<CameraControlProps> = ({ zoom }) => {
  const { camera, gl } = useThree();
  const controlsRef = useRef<OrbitControls | null>(null);

  useEffect(() => {
    const subs: Subscription[] = [];
    subs.push(
      AppEvents.zoomIn.subscribe(() => {
        camera.zoom = camera.zoom * DEFAULT_ZOOM_IN_CONSTANT;
      }),
    );
    subs.push(
      AppEvents.zoomOut.subscribe(() => {
        camera.zoom = camera.zoom * DEFAULT_ZOOM_OUT_CONSTANT;
      }),
    );
    return () => {
      for (const sub of subs) {
        sub.unsubscribe();
      }
    };
  }, [camera]);

  useEffect(() => {
    const controls = new OrbitControls(camera, gl.domElement);
    controls.target = new Vector3(0, 0, -1000);
    controls.enableRotate = false;
    controls.enableDamping = false;
    controls.enableZoom = true;
    controls.mouseButtons = {
      LEFT: MOUSE.PAN,
      MIDDLE: undefined,
      RIGHT: undefined,
    };

    controlsRef.current = controls;

    return () => {
      controls.dispose();
    };
  }, [camera, gl.domElement, zoom]);

  useFrame(() => {
    if (controlsRef.current) {
      controlsRef.current.update();
    }
  });

  return null;
};
