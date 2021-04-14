import { render, waitForElementToBeRemoved } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import React from 'react';
import { CreateTaskForm } from '../../lib';

describe('CreateTaskForm', () => {
  it('smoke test', async () => {
    const root = render(<CreateTaskForm open />);
    root.getByLabelText('Start Time');
    root.getByLabelText('Priority');

    // check fields are present in clean form
    userEvent.click(root.getByLabelText('Task Type'));
    userEvent.click(root.getByRole('option', { name: 'Clean' }));
    await waitForElementToBeRemoved(() => root.getByRole('option', { name: 'Clean' }));
    root.getByLabelText('Cleaning Zone');

    // check fields are present in loop form
    userEvent.click(root.getByLabelText('Task Type'));
    userEvent.click(root.getByRole('option', { name: 'Loop' }));
    await waitForElementToBeRemoved(() => root.getByRole('option', { name: 'Clean' }));
    root.getByLabelText('Start Location');
    root.getByLabelText('Finish Location');
    root.getByLabelText('Loops');

    // check fields are present in delivery form
    userEvent.click(root.getByLabelText('Task Type'));
    userEvent.click(root.getByRole('option', { name: 'Delivery' }));
    await waitForElementToBeRemoved(() => root.getByRole('option', { name: 'Delivery' }));
    root.getByLabelText('Pickup Location');
    root.getByLabelText('Dispenser');
    root.getByLabelText('Dropoff Location');
    root.getByLabelText('Ingestor');
  });
});
