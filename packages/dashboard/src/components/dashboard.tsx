import { DoorsPanel } from './doors-panel';
import GridLayout from 'react-grid-layout';

export function Dashboard(): JSX.Element {
  return (
    <GridLayout layout={[{ i: 'a', x: 0, y: 2, w: 4, h: 4 }]} cols={12} rowHeight={30} width={1200}>
      <DoorsPanel key="a" />
    </GridLayout>
  );
}
