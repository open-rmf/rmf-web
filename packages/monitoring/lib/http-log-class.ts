import axios from 'axios';

export interface FluentdLogProps {
  logIdentifier: string;
  url: string;
}

class FluentdLog {
  logIdentifier: string;
  url: string;

  constructor(props: FluentdLogProps) {
    this.logIdentifier = props.logIdentifier;
    this.url = props.url;
  }

  public send = (data: string, moduleIdentifier: string) => {
    // Fluentd specs to receive formData
    const form = new FormData();
    const logIdentifier = moduleIdentifier
      ? `${this.logIdentifier}:${moduleIdentifier}`
      : this.logIdentifier;
    form.set('json', JSON.stringify({ [logIdentifier]: data }));
    axios
      .post(this.url, form)
      .then(function (response) {
        console.log(response);
      })
      .catch(function (error) {
        console.log(error);
      });
  };
}

export default FluentdLog;
