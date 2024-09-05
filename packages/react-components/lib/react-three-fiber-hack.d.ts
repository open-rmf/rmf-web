// hack to export only the intrinsic elements that we use
import { ThreeElements } from '@react-three/fiber';

declare global {
  namespace JSX {
    interface IntrinsicElements
      extends Pick<
        ThreeElements,
        | 'mesh'
        | 'planeGeometry'
        | 'meshStandardMaterial'
        | 'meshBasicMaterial'
        | 'group'
        | 'boxGeometry'
        | 'shaderMaterial'
        | 'instancedMesh'
        | 'meshPhysicalMaterial'
        | 'pointsMaterial'
        | 'points'
      > {}
  }
}
