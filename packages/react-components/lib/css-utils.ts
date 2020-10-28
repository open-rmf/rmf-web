export function joinClasses(...classes: (string | undefined)[]): string {
  return classes.filter((value) => value).join(' ');
}

let id = 0;
export function uniqueId(): string {
  return (id++).toString();
}
