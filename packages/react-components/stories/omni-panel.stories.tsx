import { Button } from '@material-ui/core';
import { Story } from '@storybook/react';
import React from 'react';
import { OmniPanel, OmniPanelView, StackNavigator } from '../lib';

export default {
  title: 'Omni Panel',
};

const SimpleOmniPanel = (): JSX.Element => {
  const [view, setView] = React.useState<number | string>(0);
  const stack = React.useMemo(() => new StackNavigator<number>(0), []);

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
      <OmniPanelView viewId={0}>
        <div>
          <Button variant="outlined" onClick={() => (stack.push(1), setView(1))}>
            Panel A
          </Button>
        </div>
        <div>
          <Button variant="outlined" onClick={() => (stack.push(2), setView(2))}>
            Panel B
          </Button>
        </div>
        <div>
          <Button variant="outlined" onClick={() => (stack.push(3), setView(3))}>
            Panel C
          </Button>
        </div>
      </OmniPanelView>
      <OmniPanelView viewId={1}>
        <Button variant="outlined" onClick={() => setView(stack.pop())}>
          Back
        </Button>
      </OmniPanelView>
      <OmniPanelView viewId={2}>
        <Button variant="outlined" onClick={() => setView(stack.pop())}>
          Back
        </Button>
      </OmniPanelView>
      <OmniPanelView viewId={3}>
        <Button variant="outlined" onClick={() => setView(stack.pop())}>
          Back
        </Button>
      </OmniPanelView>
    </OmniPanel>
  );
};

export const SimplePanel: Story = (args) => {
  return <SimpleOmniPanel {...args} />;
};
