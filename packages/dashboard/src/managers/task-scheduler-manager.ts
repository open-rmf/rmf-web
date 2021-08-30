import axios from 'axios';
import { ScheduledTask, TaskRule } from 'react-components';
export const request = async (
  url: string,
  params: {
    offset: number;
    limit: number;
  },
): Promise<any> => {
  try {
    const response = await axios.get(url, {
      params: {
        offset: params.offset,
        limit: params.limit,
      },
    });
    return response.data;
  } catch (error) {
    console.error(error);
    return [];
  }
};
const BASE_PATH = 'http://localhost:8010';
export const getScheduledTasksAPI = async (offset: number): Promise<ScheduledTask[]> => {
  const url = BASE_PATH + '/task/scheduled';
  const response = await request(url, {
    offset: offset,
    limit: 500,
  });
  console.log(response);

  return response as ScheduledTask[];
};

export const getTaskRulesAPI = async (offset: number): Promise<TaskRule[]> => {
  const url = BASE_PATH + '/task/rule';
  const response = await request(url, {
    offset: offset,
    limit: 500,
  });
  console.log(response);
  return response as TaskRule[];
};

export const deleteScheduledTaskAPI = async (taskId: number): Promise<void> => {
  const url = BASE_PATH + '/task/scheduled';
  await axios.delete(url, {
    params: {
      id: taskId,
    },
  });
};

export const deleteTaskRuleAPI = async (ruleId: number): Promise<void> => {
  const url = BASE_PATH + '/task/rule';
  await axios.delete(url, {
    params: {
      id: ruleId,
    },
  });
};

type TaskRuleCreation = Omit<TaskRule, 'id'>;
export const createTaskRuleAPI = async (payload: TaskRuleCreation) => {
  const url = BASE_PATH + '/task/rule';

  try {
    const response = await axios.post(url, {
      data: payload,
    });
    return response.data;
  } catch (error) {
    console.error(error);
    return [];
  }
};
