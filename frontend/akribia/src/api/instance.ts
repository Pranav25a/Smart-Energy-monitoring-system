import axios from 'axios';


const baseUrl = "http://192.168.25.70:8090/"


const get = <T>(
  endpoint: string,
  params?: Record<string, any>,
): Promise<T> =>
  new Promise(async (resolve, reject) => {
    return axios
      .get(baseUrl + endpoint, {
        params,
        data: {},
      })
      .then((response) => resolve(response.data))
      .catch((error) => {
        reject(error);
      });
  });

const post = <T>(
  endpoint: string,
  data: object,
  contentType?: string,
  params?: Record<string, any>,
): Promise<T> =>
  new Promise(async (resolve, reject) => {
    axios(baseUrl + endpoint, {
      method: 'POST',
      data: contentType === 'multipart/form-data' ? data : JSON.stringify(data),
      params,
    })
      .then(async (response) => {
        const content = await response.data;
        return typeof content === 'string'
          ? JSON.parse(content === '' ? JSON.stringify({}) : content)
          : content;
      })
      .then((data) => {
        resolve(data);
      })
      .catch((error) => {
        reject(error);
      });
  });

const put = <T>(
  endpoint: string,
  data: object,
  params?: Record<string, any>,
  contentType?: string,
): Promise<T> =>
  new Promise(async (resolve, reject) => {
    axios(baseUrl + endpoint, {
      method: 'PUT',
      data: contentType === 'multipart/form-data' ? data : JSON.stringify(data),
      params,
    })
      .then(async (response) => {
        const content = await response.data;
        return typeof content === 'string'
          ? JSON.parse(content === '' ? JSON.stringify({}) : content)
          : content;
      })
      .then((data: any) => {
        if (data.error) {
          reject(data);
        } else {
          resolve(data);
        }
      })
      .catch((error) => {
        reject(error);
      });
  });

const remove = (
  endpoint: string,
  params?: Record<string, any>,
  headers?: Record<string, string | number | boolean>,
): Promise<void> =>
  new Promise(async (resolve, reject) => {
    axios
      .delete(baseUrl + endpoint, {
        params,
        headers,
      })
      .then(() => {
        resolve();
      })
      .catch((error) => {
        reject(error);
      });
  });

const Api = {
  get,
  post,
  put,
  remove,
};

export default Api;
