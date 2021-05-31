import { render, RenderResult, waitForElementToBeRemoved, within } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import { CleanTaskDescription, LoopTaskDescription } from 'api-client';
import React from 'react';
import * as RmfModels from 'rmf-models';
import { CreateTaskForm } from '../../lib';
import { makeSubmitTask } from './utils';

const getTaskTypeEl = (root: RenderResult) => root.getByLabelText('Task Type');

describe('CreateTaskForm', () => {
  it('check fields are present', async () => {
    const root = render(<CreateTaskForm open />);
    root.getByLabelText('Start Time');
    root.getByLabelText('Priority');

    // check fields are present in clean form
    userEvent.click(getTaskTypeEl(root));
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
    const root = render(<CreateTaskForm open submitTasks={spy} />);
    userEvent.click(root.getByLabelText('Submit'));
    expect(spy).toHaveBeenCalledTimes(1);
  });

  it('onFail is called when submitTask fails', async () => {
    const submitSpy = jasmine.createSpy().and.rejectWith(new Error('error!!'));
    const failSpy = jasmine.createSpy();
    const root = render(<CreateTaskForm open submitTasks={submitSpy} onFail={failSpy} />);
    userEvent.click(root.getByLabelText('Submit'));
    await new Promise((res) => setTimeout(res, 0));
    expect(failSpy).toHaveBeenCalledTimes(1);
  });

  it('tasksFromFile is called when select file button is clicked', () => {
    const spy = jasmine.createSpy().and.resolveTo([]);
    const root = render(<CreateTaskForm open tasksFromFile={spy} />);
    userEvent.click(root.getByLabelText('Select File'));
    expect(spy).toHaveBeenCalledTimes(1);
  });

  describe('task list', () => {
    const mount = () => {
      const task1 = makeSubmitTask();
      task1.description = { cleaning_zone: 'clean' } as CleanTaskDescription;
      task1.task_type = RmfModels.TaskType.TYPE_CLEAN;
      const task2 = makeSubmitTask();
      task2.description = {
        start_name: 'start',
        finish_name: 'finish',
        num_loops: 2,
      } as LoopTaskDescription;
      task2.task_type = RmfModels.TaskType.TYPE_LOOP;
      const tasksFromFile = () => Promise.resolve([task1, task2]);
      const root = render(<CreateTaskForm open tasksFromFile={tasksFromFile} />);
      userEvent.click(root.getByLabelText('Select File'));

      const getTaskList = async () => root.findByLabelText('Tasks List');
      const getTaskItems = async () => within(await getTaskList()).getAllByRole('listitem');

      return {
        root,
        q: {
          getTaskList,
          getTaskItems,
        },
      };
    };

    it('is shown when file is imported', async () => {
      const { q } = mount();
      const taskItems = await q.getTaskItems();
      expect(taskItems.length).toBe(2);
    });

    it('clicking on item updates the form', async () => {
      const { root, q } = mount();
      const tasks = await q.getTaskItems();
      expect(tasks.length).toBeGreaterThan(0);
      userEvent.click(tasks[0]);
      within(getTaskTypeEl(root)).getByText('Clean');
      userEvent.click(tasks[1]);
      within(getTaskTypeEl(root)).getByText('Loop');
    });
  });
});
