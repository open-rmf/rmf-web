import axios from 'axios';
import { DefaultReportQueryPayload } from 'react-components';

export const getLogData = async (
  url: string,
  params: DefaultReportQueryPayload,
  token: string | undefined | null,
) => {
  try {
    const response = await axios.get(url, {
      params: {
        toLogDate: params.toLogDate ? params.toLogDate.format() : null,
        fromLogDate: params.fromLogDate ? params.fromLogDate.format() : null,
        offset: params.offset,
        limit: params.limit,
      },
      headers: {
        Authorization: 'Bearer ' + token,
      },
    });
    return response.data;
  } catch (error) {
    console.error(error);
    return [];
  }
};
