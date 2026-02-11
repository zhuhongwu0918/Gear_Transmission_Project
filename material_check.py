# -*- coding: utf-8 -*-
"""
材料强度校核模块
提供齿轮材料选择、弯曲应力计算和强度校核功能
"""

import math
from typing import Dict, Tuple, List, Optional
from dataclasses import dataclass


@dataclass
class Material:
    """材料数据类"""
    name: str
    density: float       # kg/mm³
    sigma_f: float       # 弯曲疲劳极限 (MPa)
    sf_min_ratio: float  # 最小齿根厚/模数比
    safety_factor: float
    color: str = ''


@dataclass
class MaterialCheckResult:
    """材料校核结果"""
    material_key: str
    name: str
    max_stress: float
    allow_stress: float
    strength_ok: bool
    sf_ratio_ok: bool
    suitable: bool
    safety_margin: float
    density: float


class MaterialDatabase:
    """材料数据库"""
    
    DEFAULT_MATERIALS = {
        'steel': Material(
            name='合金钢 (20CrMnTi)',
            density=7.85e-6,
            sigma_f=600,
            sf_min_ratio=0.3,
            safety_factor=2.0,
            color='金属灰'
        ),
        'peek': Material(
            name='PEEK (聚醚醚酮)',
            density=1.32e-6,
            sigma_f=90,
            sf_min_ratio=0.8,
            safety_factor=2.5,
            color='米黄色'
        ),
        'pom': Material(
            name='POM (聚甲醛)',
            density=1.41e-6,
            sigma_f=65,
            sf_min_ratio=1.0,
            safety_factor=3.0,
            color='白色/黑色'
        ),
        'nylon': Material(
            name='尼龙66 (PA66)',
            density=1.15e-6,
            sigma_f=70,
            sf_min_ratio=0.9,
            safety_factor=2.8,
            color='乳白色'
        )
    }
    
    def __init__(self, materials: Optional[Dict[str, Material]] = None):
        """
        初始化材料数据库
        
        Args:
            materials: 自定义材料字典，默认使用内置材料
        """
        self.materials = materials or self.DEFAULT_MATERIALS.copy()
    
    def get(self, key: str) -> Optional[Material]:
        """获取材料"""
        return self.materials.get(key)
    
    def add(self, key: str, material: Material) -> None:
        """添加新材料"""
        self.materials[key] = material
    
    def keys(self) -> List[str]:
        """获取所有材料键名"""
        return list(self.materials.keys())
    
    def __contains__(self, key: str) -> bool:
        """检查是否包含某材料"""
        return key in self.materials
    
    def __getitem__(self, key: str) -> Material:
        """获取材料"""
        return self.materials[key]


class StrengthChecker:
    """强度校核器"""
    
    def __init__(self, pressure_angle: float = 20.0):
        """
        初始化强度校核器
        
        Args:
            pressure_angle: 压力角（度），默认20度
        """
        self.pressure_angle = pressure_angle
        self._sf_factor = None  # 缓存齿根厚系数
    
    @property
    def sf_factor(self) -> float:
        """齿根厚系数（缓存）"""
        if self._sf_factor is None:
            alpha = math.radians(self.pressure_angle)
            self._sf_factor = math.pi / 2 * math.cos(alpha) - 2 * 1.25 * math.tan(alpha)
        return self._sf_factor
    
    def calculate_bending_stress(
        self,
        m: float,
        z: int,
        width: float,
        torque: float,
        Y_S: float = 1.0,
        Y_epsilon: float = 0.7
    ) -> Tuple[float, float]:
        """
        计算齿根弯曲应力
        
        公式: σ_F = (2 * T * Y_F * Y_S * Y_ε) / (b * m² * z)
        
        Args:
            m: 模数 (mm)
            z: 齿数
            width: 齿宽 (mm)
            torque: 扭矩 (N·m)
            Y_S: 应力修正系数，默认1.0
            Y_epsilon: 重合度系数，默认0.7
            
        Returns:
            (sigma_F, Y_F): 弯曲应力(MPa)和齿形系数
        """
        if z < 12 or width <= 0 or m <= 0:
            return float('inf'), 0.0
        
        # 齿形系数（经验公式）
        Y_F = 2.1 + 3.5 / z
        
        # 扭矩转 N·mm
        T = torque * 1000
        
        # 弯曲应力
        sigma_F = (2 * T * Y_F * Y_S * Y_epsilon) / (width * m**2 * z)
        
        return sigma_F, Y_F
    
    def check_material(
        self,
        material: Material,
        m1: float,
        m2: float,
        z1: int, z2: int, z3: int, z4: int,
        width1: float,
        width2: float,
        torque1: float,
        torque2: float
    ) -> MaterialCheckResult:
        """
        校核单个材料是否满足强度要求
        
        Args:
            material: 材料对象
            m1, m2: 第一/二级模数
            z1, z2, z3, z4: 各齿轮齿数
            width1, width2: 第一/二级齿宽
            torque1: 第一级输入扭矩 (N·m)
            torque2: 第二级输入扭矩 (N·m)
            
        Returns:
            MaterialCheckResult: 校核结果
        """
        # 计算各级弯曲应力
        sigma_F1, _ = self.calculate_bending_stress(m1, z1, width1, torque1)
        sigma_F2, _ = self.calculate_bending_stress(m1, z2, width1, torque1 * z2 / z1)
        sigma_F3, _ = self.calculate_bending_stress(m2, z3, width2, torque2)
        sigma_F4, _ = self.calculate_bending_stress(m2, z4, width2, torque2 * z4 / z3)
        
        # 最大弯曲应力
        max_sigma_F = max(sigma_F1, sigma_F2, sigma_F3, sigma_F4)
        
        # 许用弯曲应力
        sigma_F_allow = material.sigma_f / material.safety_factor
        
        # 齿根厚检查
        sf1 = m1 * self.sf_factor
        sf2 = m2 * self.sf_factor
        sf_ratio_ok = (
            (sf1 / m1 >= material.sf_min_ratio) and 
            (sf2 / m2 >= material.sf_min_ratio)
        )
        
        # 强度检查
        strength_ok = max_sigma_F <= sigma_F_allow
        
        # 综合检查
        is_suitable = strength_ok and sf_ratio_ok
        
        # 安全裕度
        safety_margin = sigma_F_allow / max_sigma_F if max_sigma_F > 0 else float('inf')
        
        return MaterialCheckResult(
            material_key='',
            name=material.name,
            max_stress=max_sigma_F,
            allow_stress=sigma_F_allow,
            strength_ok=strength_ok,
            sf_ratio_ok=sf_ratio_ok,
            suitable=is_suitable,
            safety_margin=safety_margin,
            density=material.density
        )
    
    def select_best_material(
        self,
        db: MaterialDatabase,
        m1: float,
        m2: float,
        z1: int, z2: int, z3: int, z4: int,
        width1: float,
        width2: float,
        torque1: float,
        torque2: float,
        mode: str = 'auto'
    ) -> Tuple[str, Dict[str, MaterialCheckResult]]:
        """
        选择最优材料
        
        Args:
            db: 材料数据库
            m1, m2: 模数
            z1-z4: 齿数
            width1, width2: 齿宽
            torque1, torque2: 扭矩 (N·m)
            mode: 'auto'自动选择，或指定材料键名
            
        Returns:
            (selected_key, results_dict): 选中材料的键名和所有校核结果
        """
        results = {}
        suitable_materials = []
        
        # 校核所有材料
        for key in db.keys():
            material = db[key]
            result = self.check_material(
                material, m1, m2, z1, z2, z3, z4,
                width1, width2, torque1, torque2
            )
            result.material_key = key
            results[key] = result
            
            if result.suitable:
                suitable_materials.append((key, result))
        
        # 指定材料模式
        if mode != 'auto':
            return mode, results
        
        # 自动选择：按密度排序，优先轻量材料
        if suitable_materials:
            suitable_materials.sort(key=lambda x: x[1].density)
            selected_key = suitable_materials[0][0]
        else:
            # 没有材料满足，选强度最高的钢
            selected_key = 'steel'
        
        return selected_key, results


# ==================== 便捷函数 ====================

def quick_check(
    m1: float, m2: float,
    z1: int, z2: int, z3: int, z4: int,
    width_factor1: float, width_factor2: float,
    torque_motor: float,
    ratio1: float,
    mode: str = 'auto'
) -> Tuple[bool, str, float]:
    """
    快速校核材料强度
    
    Args:
        m1, m2: 模数
        z1-z4: 齿数
        width_factor1, width_factor2: 齿宽系数
        torque_motor: 电机扭矩 (N·m)
        ratio1: 第一级传动比
        mode: 'auto'或指定材料
        
    Returns:
        (is_ok, material_name, max_stress): 是否通过、材料名、最大应力
    """
    db = MaterialDatabase()
    checker = StrengthChecker()
    
    width1 = width_factor1 * m1
    width2 = width_factor2 * m2
    torque1 = torque_motor
    torque2 = torque1 * ratio1 * 0.95
    
    selected_key, results = checker.select_best_material(
        db, m1, m2, z1, z2, z3, z4,
        width1, width2, torque1, torque2, mode
    )
    
    # 检查选中材料是否满足
    result = results[selected_key]
    if not result.suitable and mode == 'auto':
        # 尝试其他材料
        for key, res in results.items():
            if res.suitable:
                selected_key = key
                result = res
                break
    
    return result.suitable, db[selected_key].name, result.max_stress


# ==================== 测试 ====================
if __name__ == '__main__':
    print("材料校核模块测试")
    print("=" * 50)
    
    # 测试用例
    m1, m2 = 0.8, 1.0
    z1, z2, z3, z4 = 25, 50, 20, 50
    wf1, wf2 = 15, 10
    torque = 0.8
    i1 = z2 / z1
    
    ok, name, stress = quick_check(m1, m2, z1, z2, z3, z4, wf1, wf2, torque, i1)
    
    print(f"模数: m1={m1}, m2={m2}")
    print(f"齿数: z1={z1}, z2={z2}, z3={z3}, z4={z4}")
    print(f"传动比: {i1:.2f}")
    print(f"电机扭矩: {torque} N·m")
    print(f"\n校核结果:")
    print(f"  通过: {'✓' if ok else '✗'}")
    print(f"  材料: {name}")
    print(f"  最大应力: {stress:.2f} MPa")
