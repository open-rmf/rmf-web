export enum RecurrenceType {
  Once = 'Once',
  Minutely = 'Minutely',
  Hourly = 'Hourly',
  Daily = 'Daily',
  Weekly = 'Weekly',
  Monthly = 'Monthly',
  Custom = 'Custom',
}

export enum WeekDay {
  Monday = 0,
  Tuesday,
  Wednesday,
  Thursday,
  Friday,
  Saturday,
  Sunday,
}

export class RecurrentRules {
  // static readonly FREQUENCIES: (keyof typeof Frequency)[] = [
  //   'YEARLY',
  //   'MONTHLY',
  //   'WEEKLY',
  //   'DAILY',
  //   'HOURLY',
  //   'MINUTELY',
  // ];

  // static readonly YEARLY = Frequency.YEARLY;
  // static readonly MONTHLY = Frequency.MONTHLY;
  // static readonly WEEKLY = Frequency.WEEKLY;
  // static readonly DAILY = Frequency.DAILY;
  // static readonly HOURLY = Frequency.HOURLY;
  // static readonly MINUTELY = Frequency.MINUTELY;

  // static readonly MO = Days.MO
  // static readonly TU = Days.TU
  // static readonly WE = Days.WE
  // static readonly TH = Days.TH
  // static readonly FR = Days.FR
  // static readonly SA = Days.SA
  // static readonly SU = Days.SU

  // constructor() {
  //   this.rules = [];
  // }
  // addRule(rule) {}
  interval = 1;

  static getMonthName = (date: Date): string => {
    return date.toLocaleString('default', { month: 'long' });
  };

  static getDayName = (date: Date): string => {
    return new Date(date).toLocaleString('en-us', { weekday: 'long' });
  };

  static getRecurrenceTypeList = (
    date: Date | undefined,
  ): {
    key: RecurrenceType;
    value: string;
  }[] => {
    if (!date) {
      return [
        {
          key: RecurrenceType.Once,
          value: 'Once',
        },
      ];
    }

    const dayName = RecurrentRules.getDayName(date);
    // const monthAndDate = `${RecurrentRules.getMonthName(date)} ${date.getDate()}`;

    return [
      {
        key: RecurrenceType.Once,
        value: 'Once',
      },
      {
        key: RecurrenceType.Hourly,
        value: 'Hourly',
      },
      {
        key: RecurrenceType.Daily,
        value: 'Daily',
      },
      {
        key: RecurrenceType.Weekly,
        value: `Weekly on ${dayName}`,
      },
      {
        key: RecurrenceType.Monthly,
        value: `Monthly On the First ${dayName}`,
      },
      {
        key: RecurrenceType.Custom,
        value: 'Custom..',
      },
    ];
  };

  static getBasicRecurrenceTypeList = (): {
    key: RecurrenceType;
    value: string;
  }[] => {
    return [
      {
        key: RecurrenceType.Minutely,
        value: 'Minutes(s)',
      },
      {
        key: RecurrenceType.Hourly,
        value: 'Hour(s)',
      },
      {
        key: RecurrenceType.Daily,
        value: 'Day(s)',
      },
      {
        key: RecurrenceType.Weekly,
        value: `Week(s)`,
      },
      {
        key: RecurrenceType.Monthly,
        value: `Month(s)`,
      },
    ];
  };
}

export default RecurrentRules;
