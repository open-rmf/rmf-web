import { render, waitForElementToBeRemoved } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import React from 'react';
import { CreateTaskForm } from '../../lib';

describe('CreateTaskForm', () => {
  it('check fields are present', async () => {
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

  it('onCancelClick is called when cancel button is clicked', () => {
    const spy = jasmine.createSpy();
    const root = render(<CreateTaskForm open onCancelClick={spy} />);
    userEvent.click(root.getByLabelText('Cancel'));
    expect(spy).toHaveBeenCalledTimes(1);
  });

  it('submitTask is called when form is submitted', () => {
    const spy = jasmine.createSpy().and.resolveTo(undefined);
    const root = render(<CreateTaskForm open submitTask={spy} />);
    userEvent.click(root.getByLabelText('Submit'));
    expect(spy).toHaveBeenCalledTimes(1);
  });

  it('onFail is called when submitTask fails', async () => {
    const submitSpy = jasmine.createSpy().and.rejectWith(new Error('error!!'));
    const failSpy = jasmine.createSpy();
    const root = render(<CreateTaskForm open submitTask={submitSpy} onFail={failSpy} />);
    userEvent.click(root.getByLabelText('Submit'));
    await new Promise((res) => setTimeout(res, 0));
    expect(failSpy).toHaveBeenCalledTimes(1);
  });

  it('onUploadFileClick is called when upload file button is clicked', () => {
    const spy = jasmine.createSpy();
    const root = render(<CreateTaskForm open batchMode onUploadFileClick={spy} />);
    userEvent.click(root.getByLabelText('Upload File'));
    expect(spy).toHaveBeenCalledTimes(1);
  });
});
