import { Notification } from 'react-components';

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

  makeNotification(): Notification[] {
    return [
      {
        time: this.getDate(),
        error: 'coke_ingestor not sending states',
        severity: 'High',
      },
      {
        time: this.getDate(),
        error: 'Lift is on fire',
        severity: 'High',
      },
      {
        time: this.getDate(),
        error: 'Trajectory conflict with robot B and robot C',
        severity: 'Medium',
      },
      {
        time: this.getDate(),
        error: 'Trajectory conflict with robot B and robot C',
        severity: 'Medium',
      },
      {
        time: this.getDate(),
        error: 'Trajectory conflict with robot B and robot C',
        severity: 'Medium',
      },
      {
        time: this.getDate(),
        error: 'Robot D not moving for over 10 seconds',
        severity: 'Low',
      },
      {
        time: this.getDate(),
        error: 'Lift is offline',
        severity: 'Low',
      },
    ];
  }

  // getNotifications(interval: number) {
  //   const notifications = this.makeNotification();
  //   return notifications.slice(0, interval);
  // }

  // pushNotifications(notificationsMessage: string) {
  //   Notification.requestPermission((result) => {
  //     if (result === 'granted') {
  //       const options = {
  //         body: notificationsMessage,
  //         vibrate: [200, 100, 200, 100, 200, 100, 400],
  //       };
  //       if (navigator && navigator.serviceWorker) {
  //         navigator.serviceWorker.ready.then((registration) => {
  //           if (registration && registration.showNotification) {
  //             registration.showNotification('rmf-web notificatons', options);
  //           }
  //         });
  //       }
  //     }
  //   });
  // }
}
