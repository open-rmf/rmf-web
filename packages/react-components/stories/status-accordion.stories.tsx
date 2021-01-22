import React from 'react';
import { Meta, Story } from '@storybook/react';
import { StatusAccordion, StatusIndicator, StatusPanelProps } from '../lib';

export default {
  title: 'Status Accordion',
  component: StatusAccordion,
} as Meta;

const makeStatusData = (): StatusIndicator => {
  return {
    doors: {
      door1: { state: true },
      door2: { state: false },
      door3: { state: true },
    },
    lifts: {
      lift1: { state: false },
      lift2: { state: true },
      lift3: { state: true },
      lift4: { state: true },
    },
    robots: {
      robot1: { state: true },
      robot2: { state: true },
      robot3: { state: true },
      robot4: { state: true },
    },
    dispensers: {
      dispenser1: { state: false },
      dispenser2: { state: false },
      dispenser3: { state: false },
      dispenser4: { state: false },
    },
  };
};

const StatusAccordionBuilder = (props: StatusPanelProps): JSX.Element => {
  const { statusIndicators } = props;
  const [itemIndicator, setItemIndicator] = React.useState<{ [key: string]: boolean }>({});

  React.useEffect(() => {
    const initialItemIndicator: { [key: string]: boolean } = {
      doors: true,
      lifts: true,
      robots: true,
      dispensers: true,
    };
    Object.keys(statusIndicators).forEach((category) => {
      Object.keys(statusIndicators[category]).forEach((item) => {
        if (!statusIndicators[category][item].state) {
          initialItemIndicator[category] = false;
          setItemIndicator(initialItemIndicator);
        }
      });
    });
  }, [statusIndicators]);

  return <StatusAccordion statusIndicators={makeStatusData()} itemIndicator={itemIndicator} />;
};

export const StatusAccordionDisplay: Story = () => {
  return (
    <>
      <StatusAccordionBuilder statusIndicators={makeStatusData()} />
    </>
  );
};
