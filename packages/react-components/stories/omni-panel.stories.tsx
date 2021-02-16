import { Button } from '@material-ui/core';
import { Story } from '@storybook/react';
import React from 'react';
import { OmniPanel, OmniPanelView, useStackNavigator } from '../lib';

export default {
  title: 'Omni Panel',
};

const SimpleOmniPanel = (): JSX.Element => {
  const [stack, stackDispatch] = useStackNavigator([0], 0);

  return (
    <OmniPanel
      stack={stack}
      style={{
        width: 500,
        height: 200,
        border: '1px solid black',
        borderTopLeftRadius: 16,
        borderTopRightRadius: 16,
      }}
      onBack={() => stackDispatch.pop()}
      onHome={() => stackDispatch.home()}
    >
      <OmniPanelView viewId={0}>
        <div>
          <Button variant="outlined" onClick={() => stackDispatch.push(1)}>
            Panel A
          </Button>
        </div>
        <div>
          <Button variant="outlined" onClick={() => stackDispatch.push(2)}>
            Panel B
          </Button>
        </div>
        <div>
          <Button variant="outlined" onClick={() => stackDispatch.push(3)}>
            Panel C
          </Button>
        </div>
        <div>
          <Button variant="outlined" onClick={() => stackDispatch.push(4)}>
            Panel D
          </Button>
        </div>
        <div>
          <Button variant="outlined" onClick={() => stackDispatch.push(5)}>
            Panel E
          </Button>
        </div>
      </OmniPanelView>
      <OmniPanelView viewId={1}>
        <Button variant="outlined" onClick={() => stackDispatch.pop()}>
          Back
        </Button>
      </OmniPanelView>
      <OmniPanelView viewId={2}>
        <Button variant="outlined" onClick={() => stackDispatch.pop()}>
          Back
        </Button>
      </OmniPanelView>
      <OmniPanelView viewId={3}>
        <Button variant="outlined" onClick={() => stackDispatch.pop()}>
          Back
        </Button>
      </OmniPanelView>
      <OmniPanelView viewId={4}>
        <Button variant="outlined" onClick={() => stackDispatch.pop()}>
          Back
        </Button>
      </OmniPanelView>
      <OmniPanelView viewId={5}>
        <Button variant="outlined" onClick={() => stackDispatch.pop()}>
          Back
        </Button>
      </OmniPanelView>
    </OmniPanel>
  );
};

export const SimplePanel: Story = (args) => {
  return <SimpleOmniPanel {...args} />;
};
