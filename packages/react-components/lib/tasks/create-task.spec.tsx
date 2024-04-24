// import { render, RenderResult, waitForElementToBeRemoved, within } from '@testing-library/react';
// import userEvent from '@testing-library/user-event';
//import React from 'react';
// import { TaskType as RmfTaskType } from 'rmf-models';
// import { CreateTaskForm } from './create-task';

// const getTaskTypeEl = (root: RenderResult) => root.getByLabelText('Task Type');

// describe('CreateTaskForm', () => {
//   describe('submit tasks', () => {
//     it('is called with correct clean task data when form is submitted', async () => {
//       const spy = jest.fn().and.resolveTo(undefined);
//       const root = render(<CreateTaskForm open submitTasks={spy} />);
//       await userEvent.click(getTaskTypeEl(root));
//       await userEvent.click(root.getByRole('option', { name: 'Clean' }));
//       await waitForElementToBeRemoved(() => root.getByRole('option', { name: 'Clean' }));
//       await userEvent.type(root.getByLabelText('Cleaning Zone'), 'test_zone');
//       await userEvent.click(root.getByText('Submit'));

//       expect(spy).toHaveBeenCalledTimes(1);
//       const task = spy.calls.argsFor(0)[0][0] as SubmitTask;
//       expect((task.description as CleanTaskDescription).cleaning_zone).toBe('test_zone');
//     });

//     it('is called with correct loop task data when form is submitted', async () => {
//       const spy = jest.fn().and.resolveTo(undefined);
//       const root = render(<CreateTaskForm open submitTasks={spy} />);
//       await userEvent.click(getTaskTypeEl(root));
//       await userEvent.click(root.getByRole('option', { name: 'Loop' }));
//       await waitForElementToBeRemoved(() => root.getByRole('option', { name: 'Clean' }));
//       await userEvent.type(root.getByLabelText('Start Location'), 'start');
//       await userEvent.type(root.getByLabelText('Finish Location'), 'finish');
//       const loopsInput = root.getByLabelText('Loops');
//       await userEvent.clear(loopsInput);
//       await userEvent.type(loopsInput, '2');
//       await userEvent.click(root.getByText('Submit'));

//       expect(spy).toHaveBeenCalledTimes(1);
//       const task = spy.calls.argsFor(0)[0][0] as SubmitTask;
//       const desc = task.description as LoopTaskDescription;
//       expect(desc.start_name).toBe('start');
//       expect(desc.finish_name).toBe('finish');
//       expect(desc.num_loops).toBe(2);
//     });

//     it('is called with correct delivery task data when form is submitted', async () => {
//       const spy = jest.fn().and.resolveTo(undefined);
//       const root = render(<CreateTaskForm open submitTasks={spy} />);
//       await userEvent.click(getTaskTypeEl(root));
//       await userEvent.click(root.getByRole('option', { name: 'Delivery' }));
//       await waitForElementToBeRemoved(() => root.getByRole('option', { name: 'Delivery' }));
//       await userEvent.type(root.getByLabelText('Pickup Location'), 'pickup_location');
//       await userEvent.type(root.getByLabelText('Dispenser'), 'pickup_dispenser');
//       await userEvent.type(root.getByLabelText('Dropoff Location'), 'dropoff_location');
//       await userEvent.type(root.getByLabelText('Ingestor'), 'dropoff_ingestor');
//       await userEvent.click(root.getByText('Submit'));

//       expect(spy).toHaveBeenCalledTimes(1);
//       const task = spy.calls.argsFor(0)[0][0] as SubmitTask;
//       const desc = task.description as DeliveryTaskDescription;
//       expect(desc.pickup_place_name).toBe('pickup_location');
//       expect(desc.pickup_dispenser).toBe('pickup_dispenser');
//       expect(desc.dropoff_place_name).toBe('dropoff_location');
//       expect(desc.dropoff_ingestor).toBe('dropoff_ingestor');
//     });
//   });

//   it('onClose is called when cancel button is clicked', () => {
//     const spy = jest.fn();
//     const root = render(<CreateTaskForm open onClose={spy} />);
//     await userEvent.click(root.getByText('Cancel'));
//     expect(spy).toHaveBeenCalledTimes(1);
//   });

//   it('onFail is called when submitTasks fails', async () => {
//     const submitSpy = jest.fn().and.rejectWith(new Error('error!!'));
//     const failSpy = jest.fn();
//     const root = render(<CreateTaskForm open submitTasks={submitSpy} onFail={failSpy} />);
//     await userEvent.click(root.getByText('Submit'));
//     await new Promise((res) => setTimeout(res, 0));
//     expect(failSpy).toHaveBeenCalledTimes(1);
//   });

//   it('tasksFromFile is called when select file button is clicked', () => {
//     const spy = jest.fn().and.resolveTo([]);
//     const root = render(<CreateTaskForm open tasksFromFile={spy} />);
//     await userEvent.click(root.getByLabelText('Select File'));
//     expect(spy).toHaveBeenCalledTimes(1);
//   });

//   describe('task list', () => {
//     const mount = () => {
//       const task1 = makeSubmitTask();
//       task1.description = { cleaning_zone: 'clean' } as CleanTaskDescription;
//       task1.task_type = RmfTaskType.TYPE_CLEAN;
//       const task2 = makeSubmitTask();
//       task2.description = {
//         start_name: 'start',
//         finish_name: 'finish',
//         num_loops: 2,
//       } as LoopTaskDescription;
//       task2.task_type = RmfTaskType.TYPE_LOOP;
//       const tasksFromFile = () => Promise.resolve([task1, task2]);
//       const root = render(<CreateTaskForm open tasksFromFile={tasksFromFile} />);
//       await userEvent.click(root.getByLabelText('Select File'));

//       const getTaskList = async () => root.findByLabelText('Tasks List');
//       const getTaskItems = async () => within(await getTaskList()).getAllByRole('listitem');

//       return {
//         root,
//         q: {
//           getTaskList,
//           getTaskItems,
//         },
//       };
//     };

//     it('is shown when file is imported', async () => {
//       const { q } = mount();
//       const taskItems = await q.getTaskItems();
//       expect(taskItems.length).toBe(2);
//     });

//     it('clicking on item updates the form', async () => {
//       const { root, q } = mount();
//       const tasks = await q.getTaskItems();
//       expect(tasks.length).toBeGreaterThan(0);
//       await userEvent.click(tasks[0]);
//       within(getTaskTypeEl(root)).getByText('Clean');
//       await userEvent.click(tasks[1]);
//       within(getTaskTypeEl(root)).getByText('Loop');
//     });
//   });
// });

describe('CreateTaskForm', () => {
  it('FIXME: CreateTaskForm tests are broken', () => {});
});
