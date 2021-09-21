import RBush, { BBox } from 'rbush';
import React from 'react';

export type { BBox } from 'rbush';

export interface Entity {
  bbox: BBox;
}

class EntityRBush extends RBush<Entity> {
  toBBox(entity: Entity): BBox {
    return entity.bbox;
  }
}

export class EntityManager {
  add(entity: Entity): Entity {
    this._entities.insert(entity);
    return entity;
  }

  remove(entity: Entity): void {
    this._entities.remove(entity);
  }

  /**
   * Get a bounding box that does not collides with any other entities.
   * @param bbox Preferred location.
   * @param searchThreshold
   * @returns
   */
  getNonColliding(bbox: BBox, searchThreshold = 0): BBox | null {
    if (!this.collides(bbox)) {
      return bbox;
    }

    const halfThreshold = searchThreshold / 2;
    const collidingEntites = this._entities.search({
      minX: bbox.minX - halfThreshold,
      minY: bbox.minY - halfThreshold,
      maxX: bbox.maxX + halfThreshold,
      maxY: bbox.maxY + halfThreshold,
    });

    let candidates: BBox[] = [];
    collidingEntites.forEach((colliding) => {
      // left
      candidates.push({
        minX: bbox.minX - (bbox.maxX - colliding.bbox.minX + 0.001),
        minY: bbox.minY,
        maxX: colliding.bbox.minX - 0.001,
        maxY: bbox.maxY,
      });
      // right
      candidates.push({
        minX: colliding.bbox.maxX + 0.001,
        minY: bbox.minY,
        maxX: bbox.maxX + (colliding.bbox.maxX - bbox.minX + 0.001),
        maxY: bbox.maxY,
      });
      // top
      candidates.push({
        minX: bbox.minX,
        minY: bbox.minY - (bbox.maxY - colliding.bbox.minY + 0.001),
        maxX: bbox.maxX,
        maxY: colliding.bbox.minY - 0.001,
      });
      // bottom
      candidates.push({
        minX: bbox.minX,
        minY: colliding.bbox.maxY + 0.001,
        maxX: bbox.maxX,
        maxY: bbox.maxY + (colliding.bbox.maxY - bbox.minY + 0.001),
      });
    });

    candidates = candidates.filter((candidate) => !this.collides(candidate));
    if (candidates.length === 0) return null;

    const preferredCenterX = bbox.minX + (bbox.maxX - bbox.minX) / 2;
    const preferredCenterY = bbox.minY + (bbox.maxY - bbox.minY) / 2;
    const distances = candidates.map((candidate) => {
      const centerX = candidate.minX + (candidate.maxX - candidate.minX) / 2;
      const centerY = candidate.minY + (candidate.maxY - candidate.minY) / 2;
      return (preferredCenterY - centerY) ** 2 + (preferredCenterX - centerX) ** 2;
    });
    let closestIdx = 0;
    let closestDistance = distances[0];
    distances.forEach((d, i) => {
      if (d < closestDistance) {
        closestDistance = d;
        closestIdx = i;
      }
    });
    return candidates[closestIdx];
  }

  /**
   * Returns true if the bounding box collidies with any other entities.
   * @param bbox
   * @returns
   */
  collides(bbox: BBox): boolean {
    return this._entities.collides(bbox);
  }

  private _entities = new EntityRBush();
}

export const EntityManagerContext = React.createContext(new EntityManager());
