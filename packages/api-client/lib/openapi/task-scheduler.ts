import axios from 'axios';
import { BASE_PATH } from './base';
export const request = async (
  url: string,
  params: {
    offset: number;
    limit: number;
  },
  // token: string | undefined | null,
): Promise<any> => {
  try {
    const response = await axios.get(url, {
      params: {
        offset: params.offset,
        limit: params.limit,
      },
      // headers: {
      //   Authorization: 'Bearer ' + token,
      // },
    });
    return response.data;
  } catch (error) {
    console.error(error);
    return [];
  }
};

export const getScheduledTasksAPI = async (offset: number) => {
  const url = BASE_PATH + '/api/task/scheduled';
  const response = await request(url, {
    offset: offset,
    limit: 500,
  });
  console.log(response);
};

export const getTaskRulesAPI = async (offset: number) => {
  const url = BASE_PATH + '/api/task/rule';
  const response = await request(url, {
    offset: offset,
    limit: 500,
  });
  console.log(response);
};

export default request;

// export const createTaskRule = async (
//   params: DefaultReportQueryPayload,
//   token: string | undefined | null,
// ) => {
//   try {
//     const response = await axios.get(url, {
//       params: {
//         offset: params.offset,
//         limit: params.limit,
//       },
//       headers: {
//         Authorization: 'Bearer ' + token,
//       },
//     });
//     return response.data;
//   } catch (error) {
//     console.error(error);
//     return [];
//   }
// };
