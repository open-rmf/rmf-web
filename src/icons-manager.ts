export type IconContextType = Record<string, Record<string, string>>

export default class IconManager {

    static getRobotIcon(icons: IconContextType, model: string): string | null {
        console.log(IconManager.getIcon(icons.robot, model))
        return icons.robot ? IconManager.getIcon(icons.robot, model) : null;
    }

    private static getIcon(icons: Record<string, string>, key: string): string | null {
        return icons.hasOwnProperty(key) ? icons[key] : null;
    }
}
