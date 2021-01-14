export default class FakeNotifications {
  getDate() {
    const date = new Date();
    const dateStr =
      ('00' + (date.getMonth() + 1)).slice(-2) +
      '/' +
      ('00' + date.getDate()).slice(-2) +
      '/' +
      date.getFullYear() +
      ' ' +
      ('00' + date.getHours()).slice(-2) +
      ':' +
      ('00' + date.getMinutes()).slice(-2) +
      ':' +
      ('00' + date.getSeconds()).slice(-2);
    return dateStr;
  }

  makeNotification() {
    return [
      {
        time: this.getDate(),
        location: 'x: 10, y: 11, L1',
        error: 'coke_ingestor not sending states',
        item: 'coke_ingestor',
      },
      {
        time: this.getDate(),
        location: 'x: 32, y: 19, L1',
        error: 'Robot is spoilt',
        item: 'robot A',
      },
      {
        time: this.getDate(),
        location: 'x: 8, y: 21, L1',
        error: 'conflict with robot C',
        item: 'robot B',
      },
      {
        time: this.getDate(),
        location: 'x: 16, y: 9, L1',
        error: 'conflict with robot B',
        item: 'robot C',
      },
    ];
  }

  getNotifications(interval: number) {
    const notifications = this.makeNotification();
    return notifications.slice(0, interval);
  }
}
