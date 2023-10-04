import { useFrame, useThree } from '@react-three/fiber';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls';
import React, { useEffect, useRef } from 'react';
import { MOUSE, Vector3 } from 'three';

interface CameraControlProps {
  zoom: number;
}

export const CameraControl: React.FC<CameraControlProps> = ({ zoom }) => {
  const { camera, gl } = useThree();
  const controlsRef = useRef<OrbitControls | null>(null);

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

    const handleWheel = (event: any) => {
      console.log(event);
    };

    gl.domElement.addEventListener('wheel', handleWheel);

    return () => {
      controls.dispose();
      gl.domElement.removeEventListener('wheel', handleWheel);
    };
  }, [camera, gl.domElement, zoom]);

  useFrame(() => {
    if (controlsRef.current) {
      controlsRef.current.update();
    }
  });

  return null;
};
