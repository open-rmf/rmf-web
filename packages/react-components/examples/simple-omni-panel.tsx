import Button from '@material-ui/core/Button';
import React from 'react';
import OmniPanel from '../lib/omni-panel';
import OmniPanelView from '../lib/omni-panel-view';
import StackNavigator from '../lib/stack-navigator';

export default function SimpleOmniPanel(): JSX.Element {
  const [view, setView] = React.useState<number | string>(0);
  const stack = React.useMemo(() => new StackNavigator(0), []);

  return (
    <OmniPanel
      view={view}
      style={{
        width: 500,
        height: 200,
        border: '1px solid black',
        borderTopLeftRadius: 16,
        borderTopRightRadius: 16,
      }}
      onBack={() => setView(stack.pop())}
      onHome={() => setView(stack.reset())}
    >
      <OmniPanelView id={0}>
        <Button variant="outlined" onClick={() => (stack.push(1), setView(1))}>
          Panel A
        </Button>
        <Button variant="outlined" onClick={() => (stack.push(2), setView(2))}>
          Panel B
        </Button>
        <Button variant="outlined" onClick={() => (stack.push(3), setView(3))}>
          Panel C
        </Button>
      </OmniPanelView>
      <OmniPanelView id={1}>
        <Button variant="outlined" onClick={() => setView(stack.pop())}>
          Back
        </Button>
      </OmniPanelView>
      <OmniPanelView id={2}>
        <Button variant="outlined" onClick={() => setView(stack.pop())}>
          Back
        </Button>
      </OmniPanelView>
      <OmniPanelView id={3}>
        <Button variant="outlined" onClick={() => setView(stack.pop())}>
          Back
        </Button>
      </OmniPanelView>
    </OmniPanel>
  );
}
