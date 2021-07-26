import { Button, makeStyles } from '@material-ui/core';
import { Story } from '@storybook/react';
import React from 'react';
import { OmniPanel } from './omni-panel';
import { OmniPanelView } from './omni-panel-view';
import { useStackNavigator } from './stack-navigator';

export default {
  title: 'Omni Panel',
};

interface SimpleOmniPanelProps {
  style?: React.CSSProperties;
}

const useStyles = makeStyles((theme) => ({
  omnipanel: {
    backgroundColor: theme.palette.background.default,
  },
}));

const SimpleOmniPanel = (props: SimpleOmniPanelProps): JSX.Element => {
  const [stack, stackDispatch] = useStackNavigator([0], 0);
  const classes = useStyles();

  return (
    <OmniPanel
      stack={stack}
      style={props.style}
      className={classes.omnipanel}
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

export const SimplePanelFixedSize: Story = (args) => {
  return (
    // add a background to visualize different theme colors properly
    <SimpleOmniPanel style={{ width: 500, height: 200 }} {...args} />
  );
};

export const SimplePanelAutoSize: Story = (args) => {
  // add a background to visualize different theme colors properly
  return <SimpleOmniPanel {...args} />;
};
