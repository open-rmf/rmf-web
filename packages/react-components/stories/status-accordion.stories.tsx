import React from 'react';
import { Meta, Story } from '@storybook/react';
import { StatusAccordion } from '../lib';

export default {
  title: 'Status Accordion',
  component: StatusAccordion,
} as Meta;

export const StatusAccordionDisplay: Story = () => {
  return (
    <>
      <StatusAccordion />
    </>
  );
};
