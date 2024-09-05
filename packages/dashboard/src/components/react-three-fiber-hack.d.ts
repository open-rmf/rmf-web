import '../../../react-components/lib/react-three-fiber-hack';

// hack to export only the intrinsic elements that we use
import { ThreeElements } from '@react-three/fiber';

declare global {
  namespace JSX {
    interface IntrinsicElements extends Pick<ThreeElements, 'ambientLight'> {}
  }
}
