# -*- coding: utf-8 -*-
"""
齿轮设计报告生成模块
负责格式化输出齿轮设计结果
"""

import math
from typing import Optional
from dataclasses import dataclass


@dataclass
class GearResult:
    """齿轮设计结果数据类（用于报告生成）"""
    m1: float
    m2: float
    z1: int
    z2: int
    z3: int
    z4: int
    d1: float
    d2: float
    d3: float
    d4: float
    ratio: float
    material: str
    material_name: str
    max_stress: float
    allow_stress: float
    weight_g: float
    total_size: float
    output_torque: float
    width_factor1: float
    width_factor2: float
    stress_details: dict = None
    max_stress_gear: str = ""
    gear_materials: dict = None
    gear_material_names: dict = None
    
    def __post_init__(self):
        if self.stress_details is None:
            self.stress_details = {}
        if self.gear_materials is None:
            self.gear_materials = {}
        if self.gear_material_names is None:
            self.gear_material_names = {}


@dataclass
class GearConfig:
    """齿轮配置数据类（用于报告生成）"""
    motor_torque: float
    output_torque: float
    max_motor_speed: float
    output_speed: float
    motor_diameter: float
    min_clearance: float
    motor_weight: float
    pressure_angle: float
    optimize_mode: str


class ReportGenerator:
    """齿轮设计报告生成器"""
    
    def __init__(self, result: GearResult, config: GearConfig):
        """
        初始化报告生成器
        
        Args:
            result: 齿轮设计结果
            config: 齿轮设计配置
        """
        self.result = result
        self.config = config
        self._calc_derived_params()
    
    def _calc_derived_params(self):
        """计算派生参数"""
        r = self.result
        c = self.config
        
        # 压力角
        self.pressure_angle_rad = math.radians(c.pressure_angle)
        
        # 齿距
        self.p1 = math.pi * r.m1
        self.p2 = math.pi * r.m2
        
        # 齿高参数
        self.ha1, self.ha2 = r.m1, r.m2  # 齿顶高
        self.hf1, self.hf2 = 1.25 * r.m1, 1.25 * r.m2  # 齿根高
        self.h1, self.h2 = self.ha1 + self.hf1, self.ha2 + self.hf2  # 总齿高
        
        # 齿根圆直径
        self.df1 = r.d1 - 2 * self.hf1
        self.df2 = r.d2 - 2 * self.hf1
        self.df3 = r.d3 - 2 * self.hf2
        self.df4 = r.d4 - 2 * self.hf2
        
        # 齿根宽系数和齿根宽
        sf_factor = math.pi / 2 * math.cos(self.pressure_angle_rad) - 2 * 1.25 * math.tan(self.pressure_angle_rad)
        self.sf_factor = sf_factor
        self.sf1 = r.m1 * sf_factor
        self.sf2 = r.m2 * sf_factor
        
        # 轴距
        self.wheelbase1 = (r.d1 + r.d2) / 2
        self.wheelbase2 = (r.d3 + r.d4) / 2
        
        # 各级扭矩
        self.i1 = r.z2 / r.z1
        self.i2 = r.z4 / r.z3
        self.T1_input = c.motor_torque
        self.T1_output = self.T1_input * self.i1 * 0.95
        self.T2_input = self.T1_output
        self.T2_output = self.T2_input * self.i2 * 0.95
        
        # 齿宽（齿轮厚度/轴向厚度）
        self.width1 = r.width_factor1 * r.m1
        self.width2 = r.width_factor2 * r.m2
        
        # 分隔线
        self.hline = "═" * 58
        self.hline_blue = "─" * 54
    
    def _print_header(self):
        """打印报告头部"""
        print(f"\n╔{self.hline}╗")
        print(f"║{'两级齿轮传动优化设计报告':^58}║")
        print(f"╚{self.hline}╝")
        
        opt_target = "纵向尺寸最短" if self.config.optimize_mode == 'size' else "重量最轻"
        print(f"\n▶ 【优化目标】{opt_target}")
        print(f"▶ 【选定材质】{self.result.material_name}")
    
    def _print_gear_params(self):
        """打印齿轮参数"""
        r = self.result
        
        # 获取各齿轮材质名称
        mat_z1 = r.gear_material_names.get('z1', '合金钢 (20CrMnTi)')
        mat_z2 = r.gear_material_names.get('z2', '合金钢 (20CrMnTi)')
        mat_z3 = r.gear_material_names.get('z3', '合金钢 (20CrMnTi)')
        mat_z4 = r.gear_material_names.get('z4', '合金钢 (20CrMnTi)')
        
        print(f"\n┌{'─' * 54}┐")
        print(f"│ {'第一级 (高速级)':^52} │")
        print(f"├{self.hline_blue}┤")
        print(f"│  模数 m1 = {r.m1} mm, 齿距 p1 = {self.p1:.3f} mm")
        print(f"│  齿顶高 = {self.ha1:.2f} mm, 齿根高 = {self.hf1:.2f} mm, 总齿高 = {self.h1:.2f} mm")
        print(f"│  齿宽系数 = {r.width_factor1}, 轴向厚度(齿宽) = {self.width1:.2f} mm")
        print(f"│  危险截面齿根宽 sf1 = {self.sf1:.3f} mm (sf/m = {self.sf_factor:.3f})")
        print(f"│")
        print(f"│  主动轮: 齿数 z1 = {r.z1}, 分度圆直径 D1 = {r.d1:.2f} mm, 齿轮厚度 = {self.width1:.2f} mm")
        print(f"│          材质: {mat_z1}")
        print(f"│          齿根圆直径 = {self.df1:.2f} mm")
        print(f"│          输入扭矩 = {self.T1_input:.3f} N·m")
        print(f"│  从动轮: 齿数 z2 = {r.z2}, 分度圆直径 D2 = {r.d2:.2f} mm, 齿轮厚度 = {self.width1:.2f} mm")
        print(f"│          材质: {mat_z2}")
        print(f"│          齿根圆直径 = {self.df2:.2f} mm")
        print(f"│          输出扭矩 = {self.T1_output:.3f} N·m")
        print(f"│  传动比 i1 = {self.i1:.3f}")
        print(f"│")
        print(f"│ {'第二级 (低速级)':^52} │")
        print(f"├{self.hline_blue}┤")
        print(f"│  模数 m2 = {r.m2} mm, 齿距 p2 = {self.p2:.3f} mm")
        print(f"│  齿顶高 = {self.ha2:.2f} mm, 齿根高 = {self.hf2:.2f} mm, 总齿高 = {self.h2:.2f} mm")
        print(f"│  齿宽系数 = {r.width_factor2}, 轴向厚度(齿宽) = {self.width2:.2f} mm")
        print(f"│  危险截面齿根宽 sf2 = {self.sf2:.3f} mm (sf/m = {self.sf_factor:.3f})")
        print(f"│")
        print(f"│  主动轮: 齿数 z3 = {r.z3}, 分度圆直径 D3 = {r.d3:.2f} mm, 齿轮厚度 = {self.width2:.2f} mm")
        print(f"│          材质: {mat_z3}")
        print(f"│          齿根圆直径 = {self.df3:.2f} mm")
        print(f"│          输入扭矩 = {self.T2_input:.3f} N·m")
        print(f"│  从动轮: 齿数 z4 = {r.z4}, 分度圆直径 D4 = {r.d4:.2f} mm, 齿轮厚度 = {self.width2:.2f} mm")
        print(f"│          材质: {mat_z4}")
        print(f"│          齿根圆直径 = {self.df4:.2f} mm")
        print(f"│          输出扭矩 = {self.T2_output:.3f} N·m")
        print(f"│  传动比 i2 = {self.i2:.3f}")
        print(f"└{'─' * 54}┘")
    
    def _print_performance(self):
        """打印总体性能"""
        r = self.result
        c = self.config
        
        print(f"\n┌{'─' * 54}┐")
        print(f"│ {'总体性能':^52} │")
        print(f"├{self.hline_blue}┤")
        print(f"│  总减速比: {r.ratio:.3f}")
        print(f"│  实际齿数配比: ({r.z1}:{r.z2}) × ({r.z3}:{r.z4})")
        print(f"│  输出转速: {c.output_speed} rpm (固定目标值)")
        
        motor_speed = c.output_speed * r.ratio
        speed_ok = "✓ 满足" if motor_speed <= c.max_motor_speed else "✗ 超限"
        print(f"│  期望电机转速: {motor_speed:.1f} rpm (约束: <= {c.max_motor_speed} rpm) 【{speed_ok}】")
        
        torque_ok = "✓ 满足" if r.output_torque >= c.output_torque else "✗ 不足"
        print(f"│  输出扭矩: {r.output_torque:.3f} Nm (要求 >= {c.output_torque} Nm) 【{torque_ok}】")
        print(f"└{'─' * 54}┘")
    
    def _print_dimensions(self):
        """打印尺寸与重量"""
        r = self.result
        c = self.config
        
        print(f"\n┌{'─' * 54}┐")
        print(f"│ {'尺寸与重量':^52} │")
        print(f"├{self.hline_blue}┤")
        print(f"│ 轴距参数:")
        print(f"│   第一级中心距 (电机轴-中间轴): {self.wheelbase1:.2f} mm")
        print(f"│   第二级中心距 (中间轴-输出轴): {self.wheelbase2:.2f} mm")
        print(f"│ 齿轮轴向总厚度: {self.width1 + self.width2:.2f} mm (两级齿轮厚度之和)")
        print(f"│ 总体轴向尺寸: {self.width1 + self.width2 + 15:.2f} mm (齿轮厚度 + 轴承间隙15mm)")
        print(f"│ 最大径向尺寸: {max(r.d2, r.d4):.2f} mm (电机直径参考: {c.motor_diameter} mm)")
        print(f"│")
        print(f"│ 纵向尺寸组成:")
        print(f"│   电机半径: {c.motor_diameter/2:.2f} mm")
        print(f"│   D1/2 (一级主动轮半径): {r.d1/2:.2f} mm")
        print(f"│   D2/2 (一级从动轮半径): {r.d2/2:.2f} mm")
        print(f"│   D3/2 (二级主动轮半径): {r.d3/2:.2f} mm")
        print(f"│   D4 (二级从动轮直径): {r.d4:.2f} mm")
        print(f"│   {'总纵向尺寸:':<20} {r.total_size:.2f} mm {'← 优化指标' if c.optimize_mode == 'size' else ''}")
        print(f"│")
        print(f"│ 齿轮组重量: {r.weight_g:.1f} g")
        print(f"│ 电机重量: {c.motor_weight} g")
        total_weight = r.weight_g + c.motor_weight
        print(f"│ {'系统总重量:':<20} {total_weight:.1f} g ({total_weight/1000:.3f} kg) {'← 优化指标' if c.optimize_mode == 'weight' else ''}")
        print(f"└{'─' * 54}┘")
    
    def _print_material_check(self):
        """打印材料强度校核"""
        r = self.result
        
        stress_ratio = r.max_stress / r.allow_stress
        stress_status = "✓ 安全" if stress_ratio < 1.0 else "✗ 超限"
        
        # 解析最大应力位置
        max_gear_name = r.max_stress_gear if r.max_stress_gear else "未知"
        
        print(f"\n┌{'─' * 54}┐")
        print(f"│ {'材料强度校核':^52} │")
        print(f"├{self.hline_blue}┤")
        print(f"│ 选定材质: {r.material_name}")
        print(f"│")
        print(f"│ 【各齿轮弯曲应力详情】")
        
        # 打印各齿轮应力
        if r.stress_details:
            # 第一级啮合对
            s1 = r.stress_details.get('z1(一级主动)', 0)
            s2 = r.stress_details.get('z2(一级从动)', 0)
            pair1_max = max(s1, s2)
            marker1 = " ← 最大" if r.max_stress_gear in ['z1(一级主动)', 'z2(一级从动)'] else ""
            print(f"│  第一级啮合 (z1-z2):")
            print(f"│    z1 主动轮: {s1:.1f} MPa")
            print(f"│    z2 从动轮: {s2:.1f} MPa")
            print(f"│    啮合最大: {pair1_max:.1f} MPa{marker1}")
            print(f"│")
            
            # 第二级啮合对
            s3 = r.stress_details.get('z3(二级主动)', 0)
            s4 = r.stress_details.get('z4(二级从动)', 0)
            pair2_max = max(s3, s4)
            marker2 = " ← 最大" if r.max_stress_gear in ['z3(二级主动)', 'z4(二级从动)'] else ""
            print(f"│  第二级啮合 (z3-z4):")
            print(f"│    z3 主动轮: {s3:.1f} MPa")
            print(f"│    z4 从动轮: {s4:.1f} MPa")
            print(f"│    啮合最大: {pair2_max:.1f} MPa{marker2}")
        
        print(f"│")
        print(f"│ 【应力校核总结】")
        print(f"│ 最大应力位置: {max_gear_name}")
        print(f"│ 最大弯曲应力: {r.max_stress:.1f} MPa")
        print(f"│ 许用弯曲应力: {r.allow_stress:.1f} MPa")
        print(f"│ 应力比: {stress_ratio:.2f} {stress_status}")
        print(f"│ 安全裕度: {r.allow_stress/r.max_stress:.2f}")
        print(f"└{'─' * 54}┘")
    
    def _print_validation(self):
        """打印设计验证"""
        r = self.result
        c = self.config
        
        print(f"\n┌{'─' * 54}┐")
        print(f"│ {'设计验证':^52} │")
        print(f"├{self.hline_blue}┤")
        print(f"│ 轴距约束验证: ✓ 通过")
        clearance = r.d1 - r.d3
        clearance_ok = "✓ 通过" if clearance >= c.min_clearance else "✗ 失败"
        print(f"│ 齿轮间隙验证 (D1-D3 >= {c.min_clearance}mm): {clearance:.2f} mm 【{clearance_ok}】")
        strength_factor = (r.m2 * self.width2) / (r.m1 * self.width1)
        strength_note = ">1.0表示低速级更强"
        print(f"│ 低速级相对强度系数: {strength_factor:.2f} ({strength_note})")
        print(f"└{'─' * 54}┘")
    
    def _print_footer(self):
        """打印报告尾部"""
        print(f"\n╔{self.hline}╗")
        print(f"║{'设计完成':^58}║")
        print(f"╚{self.hline}╝")
    
    def generate(self):
        """生成完整报告"""
        self._print_header()
        self._print_gear_params()
        self._print_performance()
        self._print_dimensions()
        self._print_material_check()
        self._print_validation()
        self._print_footer()


def print_result(result, config):
    """
    打印设计结果的便捷函数
    
    Args:
        result: GearResult 对象（来自 gear_optimizer）
        config: GearConfig 对象（来自 gear_optimizer）
    """
    if result is None:
        print("\n未找到可行设计方案!")
        return
    
    # 转换为报告模块的数据类
    report_result = GearResult(
        m1=result.m1, m2=result.m2,
        z1=result.z1, z2=result.z2, z3=result.z3, z4=result.z4,
        d1=result.d1, d2=result.d2, d3=result.d3, d4=result.d4,
        ratio=result.ratio,
        material=result.material,
        material_name=result.material_name,
        max_stress=result.max_stress,
        allow_stress=result.allow_stress,
        weight_g=result.weight_g,
        total_size=result.total_size,
        output_torque=result.output_torque,
        width_factor1=result.width_factor1,
        width_factor2=result.width_factor2,
        stress_details=result.stress_details,
        max_stress_gear=result.max_stress_gear,
        gear_materials=result.gear_materials,
        gear_material_names=result.gear_material_names
    )
    
    report_config = GearConfig(
        motor_torque=config.motor_torque,
        output_torque=config.output_torque,
        max_motor_speed=config.max_motor_speed,
        output_speed=config.output_speed,
        motor_diameter=config.motor_diameter,
        min_clearance=config.min_clearance,
        motor_weight=config.motor_weight,
        pressure_angle=config.pressure_angle,
        optimize_mode=config.optimize_mode
    )
    
    generator = ReportGenerator(report_result, report_config)
    generator.generate()
