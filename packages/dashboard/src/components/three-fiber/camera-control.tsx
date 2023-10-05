import { useFrame, useThree } from '@react-three/fiber';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls';
import React, { useEffect, useRef } from 'react';
import { MOUSE, Vector3 } from 'three';
import { AppEvents } from '../app-events';
import { debounce } from 'react-components';

interface CameraControlProps {
  zoom: number;
}

export const updateZoom = (currentZoom: number, increment: number) =>
  Math.max(0, Math.min(Infinity, currentZoom + increment));

export const CameraControl: React.FC<CameraControlProps> = ({ zoom }) => {
  const { camera, gl } = useThree();
  const controlsRef = useRef<OrbitControls | null>(null);
  const [internalZoom, setInternalZoom] = React.useState(zoom);

  const updateExternalStateDebounced = debounce((zoomValue: number) => {
    AppEvents.zoom.next(zoomValue);
  }, 10);

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
    camera.zoom = zoom;
    camera.updateProjectionMatrix();

    const handleWheel = (event: WheelEvent) => {
      const delta = event.deltaY;
      const sensitivity = 0.1;

      setInternalZoom((prevZoom) => {
        const newZoom = prevZoom - delta * sensitivity;
        return Math.max(controls.minZoom, Math.min(controls.maxZoom, newZoom));
      });

      updateExternalStateDebounced(internalZoom);
    };

    gl.domElement.addEventListener('wheel', handleWheel);

    return () => {
      controls.dispose();
      gl.domElement.removeEventListener('wheel', handleWheel);
    };
  }, [camera, gl.domElement, zoom, internalZoom, updateExternalStateDebounced]);

  useFrame(() => {
    if (controlsRef.current) {
      controlsRef.current.update();
    }
  });

  return null;
};
