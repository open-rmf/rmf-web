import { DoorsWindow } from './doors-window';
import { WindowContainer } from 'react-components';

export function Dashboard(): JSX.Element {
  return (
    <WindowContainer layout={[{ i: 'a', x: 0, y: 0, w: 4, h: 4 }]} designMode>
      <DoorsWindow key="a" title="Doors" />
    </WindowContainer>
  );
}
